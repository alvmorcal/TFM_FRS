/*  XIAO_FRS_BLE.ino
    Lee A0 (salida de tu filtro+ganancia) y transmite por BLE (Nordic UART Service)
    líneas "t_ms,adc\n" a ~SR Hz (por defecto 100 Hz). Permite cambiar SR por comando.

    Comandos por BLE (desde la app de PC):
      - SR=<N>   (5..500)   p.ej. "SR=200"
      - PING     -> "PONG"
      - ID?      -> nombre del dispositivo

    Hardware:
      - Seeed XIAO nRF52840 (Pre-Soldered)
      - Entrada en A0: señal centrada en Vref (~1.65 V), 0..3.3 V sin saturar
*/

#include <bluefruit.h>

// ==== CONFIGURACIÓN ====
const char* DEVICE_NAME = "XIAO-FRS";   // cambia si quieres distinguir varios
const uint8_t ADC_PIN = A0;
volatile uint16_t sample_rate_hz = 100; // 5..500 Hz
const uint8_t OVERSAMPLE = 4;           // media simple para reducir ruido
// =======================

BLEUart bleuart; // Nordic UART Service

// temporización por micros()
volatile uint32_t sample_period_us = 1000000UL / 100;
uint32_t next_sample_us = 0;

// Estado LED
void led_set(bool on) { digitalWrite(PIN_LED, on ? HIGH : LOW); }

// ---- BLE advertising ----
void startAdv(void) {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);

  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // 20ms..152.5ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0); // infinito
}

// ---- Callbacks BLE ----
void connect_callback(uint16_t) {
  led_set(true);
  Serial.println("BLE conectado.");
}

void disconnect_callback(uint16_t, uint8_t) {
  led_set(false);
  Serial.println("BLE desconectado.");
}

void update_sample_period() {
  if (sample_rate_hz < 5) sample_rate_hz = 5;
  if (sample_rate_hz > 500) sample_rate_hz = 500;
  sample_period_us = 1000000UL / sample_rate_hz;
}

void bleuart_rx_callback(uint16_t) {
  // Recibe líneas de texto con '\n'
  while (bleuart.available()) {
    String cmd = bleuart.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("SR=")) {
      uint16_t sr = cmd.substring(3).toInt();
      if (sr >= 5 && sr <= 500) {
        sample_rate_hz = sr;
        update_sample_period();
        bleuart.print(String("OK SR=") + sample_rate_hz + "\n");
      } else {
        bleuart.print("ERR SR out of range (5..500)\n");
      }
    } else if (cmd.equalsIgnoreCase("PING")) {
      bleuart.print("PONG\n");
    } else if (cmd.equalsIgnoreCase("ID?")) {
      bleuart.print(String(DEVICE_NAME) + "\n");
    } else if (cmd.length() > 0) {
      bleuart.print("ERR Unknown cmd\n");
    }
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  led_set(false);

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // ADC a 12 bits, referencia VDD (3.3 V)
  analogReadResolution(12); // 0..4095

  // BLE
  Bluefruit.begin();
  Bluefruit.setTxPower(-4); // dBm (bajo)
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  bleuart.begin();
  bleuart.setRxCallback(bleuart_rx_callback);

  startAdv();
  update_sample_period();
  next_sample_us = micros() + sample_period_us;

  led_set(true);
  Serial.println("XIAO-FRS listo. Anunciando BLE.");
}

void loop() {
  // Muestreo periódico con oversampling
  uint32_t now = micros();
  if ((int32_t)(now - next_sample_us) >= 0) {
    next_sample_us += sample_period_us;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < OVERSAMPLE; i++) {
      sum += analogRead(ADC_PIN);
      delayMicroseconds(100);
    }
    uint16_t adc = sum / OVERSAMPLE;
    uint32_t t_ms = millis();

    // Envío "t_ms,adc\n"
    bleuart.print(t_ms);
    bleuart.print(',');
    bleuart.print(adc);
    bleuart.print('\n');
  }

  // cede tiempo al stack BLE
  // (Bluefruit gestiona internamente; no es necesario llamar a loop explícito)
}
