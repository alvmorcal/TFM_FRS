"""
frs_ble_monitor.py
Monitor BLE para XIAO-FRS (Nordic UART Service).
- Auto reconexión/escaneo BLE
- Filtrado 0.2–5 Hz (scipy si está disponible; si no, fallback)
- Detección de picos con umbral MAD + RPM
- Indicador de calidad (p-p, clipping)
- Ajuste SR desde la GUI y guardado CSV

Uso:
  pip install bleak pyqtgraph numpy scipy PyQt5
  python frs_ble_monitor.py
"""

import asyncio
import threading
import sys
import time
from collections import deque

import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from bleak import BleakScanner, BleakClient

try:
    from scipy.signal import butter, lfilter
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False

# ===== Config BLE / Señal =====
DEVICE_NAME = "XIAO-FRS"  # cámbialo si renombras tu dispositivo
NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_CHAR = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (dispositivo -> PC)
NUS_RX_CHAR = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write   (PC -> dispositivo)
TARGET_SAMPLE_RATE = 100  # Hz por defecto

LOW_CUTOFF = 0.2   # Hz
HIGH_CUTOFF = 5.0  # Hz
PEAK_REFRACTORY_S = 0.25
WINDOW_SECONDS = 20
RPM_WINDOW_N = 6
DYNAMIC_THR_K = 2.0

CLIP_LO = 50
CLIP_HI = 4045

# ===== Hilo/event loop BLE =====
ble_loop = None  # event loop dedicado al hilo BLE


class FRSApp(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FRS Respiratory Monitor (BLE)")
        self.resize(1100, 680)

        v = QtWidgets.QVBoxLayout(self)

        # Barra de estado BLE
        top = QtWidgets.QHBoxLayout()
        self.status_lbl = QtWidgets.QLabel("BLE: DESCONOCIDO")
        sf = self.status_lbl.font()
        sf.setPointSize(11)
        self.status_lbl.setFont(sf)
        self.btn_reconnect = QtWidgets.QPushButton("Reconectar")
        self.btn_reconnect.clicked.connect(self.request_reconnect)
        self.devname_edit = QtWidgets.QLineEdit(DEVICE_NAME)
        self.devname_edit.setFixedWidth(220)
        top.addWidget(QtWidgets.QLabel("Dispositivo:"))
        top.addWidget(self.devname_edit)
        top.addStretch(1)
        top.addWidget(self.status_lbl)
        top.addWidget(self.btn_reconnect)
        v.addLayout(top)

        # RPM + Calidad
        self.rpm_label = QtWidgets.QLabel("RPM: --")
        f = self.rpm_label.font()
        f.setPointSize(22)
        self.rpm_label.setFont(f)

        self.quality_label = QtWidgets.QLabel("Calidad: -- | p-p: -- | clip: no")
        fq = self.quality_label.font()
        fq.setPointSize(12)
        self.quality_label.setFont(fq)

        head = QtWidgets.QHBoxLayout()
        head.addWidget(self.rpm_label)
        head.addStretch(1)
        head.addWidget(self.quality_label)
        v.addLayout(head)

        # Gráficas
        self.plot_raw = pg.PlotWidget(title="ADC bruto")
        self.plot_filt = pg.PlotWidget(title="Filtrado (0.2–5 Hz) + picos")
        for pw in (self.plot_raw, self.plot_filt):
            pw.showGrid(x=True, y=True, alpha=0.2)
        v.addWidget(self.plot_raw, 1)
        v.addWidget(self.plot_filt, 1)
        self.curve_raw = self.plot_raw.plot()
        self.curve_filt = self.plot_filt.plot()
        self.scatter_peaks = pg.ScatterPlotItem(size=10)
        self.plot_filt.addItem(self.scatter_peaks)

        # Controles
        h = QtWidgets.QHBoxLayout()
        self.btn_save = QtWidgets.QPushButton("Guardar CSV")
        self.btn_save.clicked.connect(self.save_csv)
        self.btn_sr_dn = QtWidgets.QPushButton("SR -10")
        self.btn_sr_up = QtWidgets.QPushButton("SR +10")
        self.btn_sr_dn.clicked.connect(lambda: self.send_cmd(f"SR={self.sample_rate-10}"))
        self.btn_sr_up.clicked.connect(lambda: self.send_cmd(f"SR={self.sample_rate+10}"))
        h.addWidget(self.btn_save)
        h.addStretch(1)
        h.addWidget(self.btn_sr_dn)
        h.addWidget(self.btn_sr_up)
        v.addLayout(h)

        # Buffers
        self.sample_rate = TARGET_SAMPLE_RATE
        self.window = WINDOW_SECONDS
        self.maxlen = int(self.window * self.sample_rate) + 1
        self.t_abs = deque(maxlen=self.maxlen)   # s, desde el reloj del dispositivo (millis/1000)
        self.y_raw = deque(maxlen=self.maxlen)
        self.y_flt = deque(maxlen=self.maxlen)
        self.peak_times = deque(maxlen=2048)

        # Filtro
        if HAVE_SCIPY:
            b, a = butter(2, [LOW_CUTOFF/(self.sample_rate/2), HIGH_CUTOFF/(self.sample_rate/2)], btype="band")
            self.b, self.a = b, a
            self.zi = np.zeros(max(len(a), len(b)) - 1)
        else:
            self.b = self.a = self.zi = None

        # BLE
        self.client = None
        self.connected = False
        self.want_reconnect = True  # auto-reconexión

        # Log CSV (diezmado)
        self.csv_log = []

        # Buffer de líneas (notificaciones BLE pueden llegar partidas)
        self.linebuf = ""

        # Timer de refresco GUI
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh_plots)
        self.timer.start(50)

        # Poner estado inicial
        self.set_status("BLE: ESCANEANDO...")

    # ======== GUI helpers ========
    def set_status(self, txt):
        self.status_lbl.setText(txt)

    def request_reconnect(self):
        # hace que el hilo BLE reinicie búsqueda usando el nombre actual
        self.want_reconnect = True
        self.set_status("BLE: RECONEXIÓN SOLICITADA...")
        # cerrar si hubiera cliente activo
        if self.client is not None:
            try:
                c = self.client
                self.client = None
                self.connected = False
                asyncio.run_coroutine_threadsafe(c.disconnect(), ble_loop)
            except Exception:
                pass

    # ======== BLE control desde GUI ========
    def set_ble_client(self, client):
        self.client = client
        self.connected = client is not None

    def send_cmd(self, cmd: str):
        global ble_loop
        # Normaliza SR min/max
        if cmd.startswith("SR="):
            try:
                sr = int(cmd.split("=")[1])
                if sr < 5: sr = 5
                if sr > 500: sr = 500
                cmd = f"SR={sr}"
                self.sample_rate = sr
                # actualizar filtro si es necesario
                if HAVE_SCIPY:
                    b, a = butter(2, [LOW_CUTOFF/(self.sample_rate/2), HIGH_CUTOFF/(self.sample_rate/2)], btype="band")
                    self.b, self.a = b, a
                    self.zi = np.zeros(max(len(a), len(b)) - 1)
                self.maxlen = int(self.window * self.sample_rate) + 1
                self.t_abs = deque(self.t_abs, maxlen=self.maxlen)
                self.y_raw = deque(self.y_raw, maxlen=self.maxlen)
                self.y_flt = deque(self.y_flt, maxlen=self.maxlen)
            except Exception:
                pass

        if self.client and self.connected and ble_loop is not None:
            asyncio.run_coroutine_threadsafe(
                self.client.write_gatt_char(NUS_RX_CHAR, (cmd+"\n").encode()),
                ble_loop
            )

    # ======== Archivo CSV ========
    def save_csv(self):
        if not self.csv_log:
            QtWidgets.QMessageBox.information(self, "CSV", "No hay datos todavía.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Guardar CSV", "frs_session.csv", "CSV (*.csv)")
        if not path:
            return
        try:
            import csv
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_ms", "adc", "filtered"])
                for row in self.csv_log:
                    w.writerow(row)
            QtWidgets.QMessageBox.information(self, "CSV", f"Guardado en:\n{path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "CSV", str(e))

    # ======== Procesamiento de datos ========
    def on_uart(self, _char, data: bytearray):
        # Acumula en buffer por si llegan trozos
        self.linebuf += data.decode(errors="ignore")
        *lines, self.linebuf = self.linebuf.split("\n")
        for line in lines:
            line = line.strip()
            if not line:
                continue
            # Se esperan líneas "t_ms,adc"
            try:
                t_ms_s, adc_s = line.split(",")
                t_abs = float(t_ms_s) / 1000.0
                y = float(adc_s)
            except Exception:
                # también llegan respuestas de comandos (OK SR=..., PONG, etc.)
                continue

            self.t_abs.append(t_abs)
            self.y_raw.append(y)

            # Filtrado
            yf = y
            if HAVE_SCIPY and self.b is not None:
                yf, self.zi = lfilter(self.b, self.a, np.array([y]), zi=self.zi)
                yf = float(yf[0])
            else:
                if len(self.y_raw) > 20:
                    yf = y - float(np.mean(list(self.y_raw)[-20:]))
            self.y_flt.append(yf)

            # Detección de picos
            self.detect_peaks(t_abs)

            # Log (diezmado ~20 Hz)
            if not self.csv_log or (t_abs - self.csv_log[-1][0]/1000.0) > 0.05:
                self.csv_log.append([int(t_abs*1000), int(y), float(yf)])

    def detect_peaks(self, t_now: float):
        y = np.array(self.y_flt, dtype=float)
        if len(y) < int(2*self.sample_rate):
            return
        tail = y[-int(2*self.sample_rate):]
        med = np.median(tail)
        mad = np.median(np.abs(tail - med)) + 1e-6
        thr = med + DYNAMIC_THR_K * mad

        if len(y) >= 3:
            y1, y2, y3 = y[-3], y[-2], y[-1]
            if (y2 > thr) and (y2 > y1) and (y2 > y3):
                if not self.peak_times or (t_now - self.peak_times[-1]) > PEAK_REFRACTORY_S:
                    self.peak_times.append(t_now)

        rpm = self.compute_rpm()
        self.rpm_label.setText(f"RPM: {rpm:5.1f}" if rpm is not None else "RPM: --")

    def compute_rpm(self):
        if len(self.peak_times) < 2:
            return None
        times = list(self.peak_times)[-max(RPM_WINDOW_N, 2):]
        ibis = np.diff(times)
        ibis = ibis[ibis > 0.2]  # filtra intervalos imposibles
        if len(ibis) == 0:
            return None
        return 60.0 / np.median(ibis)

    def refresh_plots(self):
        if not self.t_abs:
            return

        t_abs = np.array(self.t_abs, dtype=float)
        y = np.array(self.y_raw, dtype=float)
        yf = np.array(self.y_flt, dtype=float)

        t_end = t_abs[-1]
        t0 = t_end - self.window
        mask = t_abs >= t0
        t_rel = t_abs[mask] - t_abs[mask][0]
        y_vis = y[mask]
        yf_vis = yf[mask]

        self.curve_raw.setData(t_rel, y_vis)
        self.curve_filt.setData(t_rel, yf_vis)

        # Picos en ventana
        pts_t, pts_y = [], []
        if len(self.peak_times) > 0:
            for pt in self.peak_times:
                if t0 <= pt <= t_end:
                    tw = pt - t0
                    idx = int(np.argmin(np.abs(t_abs[mask] - pt)))
                    val = yf_vis[idx] if 0 <= idx < len(yf_vis) else np.nan
                    if np.isfinite(val):
                        pts_t.append(tw)
                        pts_y.append(val)
        self.scatter_peaks.setData(pts_t, pts_y)

        # Calidad
        self.update_quality(y_vis, yf_vis)

    def update_quality(self, y_raw_vis, y_flt_vis):
        if len(y_flt_vis) < 20:
            self.quality_label.setText("Calidad: -- | p-p: -- | clip: no")
            return
        pp = float(np.max(y_flt_vis) - np.min(y_flt_vis))
        clipped = (np.min(y_raw_vis) <= CLIP_LO) or (np.max(y_raw_vis) >= CLIP_HI)
        if clipped:
            st = "Clipping"
        elif pp < 100:
            st = "Señal baja"
        elif pp > 1200:
            st = "Riesgo de clip"
        else:
            st = "OK"
        self.quality_label.setText(f"Calidad: {st} | p-p: {pp:.0f} | clip: {'sí' if clipped else 'no'}")


# ===== BLE hilo: escaneo, conexión, reconexión =====
async def ble_worker(app: FRSApp):
    backoff = 1.5  # s
    while True:
        try:
            if not app.want_reconnect:
                await asyncio.sleep(0.5)
                continue

            app.set_status("BLE: ESCANEANDO...")
            target_name = app.devname_edit.text().strip() or DEVICE_NAME
            device = None

            # Descubre 5 s
            devices = await BleakScanner.discover(timeout=5.0)
            # match por nombre exacto primero
            for d in devices:
                if d.name == target_name:
                    device = d
                    break
            # si no, por UUID de servicio NUS
            if device is None:
                for d in devices:
                    uu = [s.lower() for s in d.metadata.get("uuids", [])] if d.metadata else []
                    if NUS_SERVICE.lower() in uu:
                        device = d
                        break

            if device is None:
                app.set_status("BLE: no encontrado (reintentando)")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 1.5, 8.0)
                continue

            app.set_status(f"BLE: conectando a {device.name} ({device.address})")
            async with BleakClient(device) as client:
                app.set_ble_client(client)
                app.want_reconnect = True  # seguimos en modo auto

                await client.start_notify(NUS_TX_CHAR, app.on_uart)
                # Ajusta SR y haz PING
                try:
                    await client.write_gatt_char(NUS_RX_CHAR, f"SR={TARGET_SAMPLE_RATE}\n".encode())
                    await client.write_gatt_char(NUS_RX_CHAR, b"PING\n")
                except Exception:
                    pass

                app.set_status(f"BLE: CONECTADO a {device.name}")
                backoff = 1.5

                # Mantén la conexión viva
                while True:
                    await asyncio.sleep(2.0)
                    # opcional: keep-alive
                    try:
                        await client.write_gatt_char(NUS_RX_CHAR, b"PING\n")
                    except Exception:
                        raise  # forzar reintento

        except Exception as e:
            app.set_ble_client(None)
            app.set_status(f"BLE: desconectado ({e}) — reintentando")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 1.5, 8.0)


def start_ble_thread(app: FRSApp):
    global ble_loop
    def _runner():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        globals()['ble_loop'] = loop
        loop.run_until_complete(ble_worker(app))
    th = threading.Thread(target=_runner, daemon=True)
    th.start()


def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = FRSApp()
    gui.show()
    start_ble_thread(gui)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

