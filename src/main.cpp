#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "Calibration.h"

// Pines segun esquema Rev C
constexpr uint8_t PIN_BOTON_RESET = 33;  // SW1 a GND, INPUT_PULLUP
constexpr uint8_t PIN_LED_ALERTA  = 32;  // GPIO32 -> R10 -> Q4 -> LED D5 (activo en ALTO)
constexpr uint8_t PIN_BUZZER      = 26;  // GPIO26 -> R8 -> Q3 -> BZ1 (activo en ALTO)
constexpr uint8_t PIN_SDA         = 21;
constexpr uint8_t PIN_SCL         = 22;

// Shunt del modulo INA219: R100 = 0.1 ohm
constexpr float SHUNT_OHMS = 0.1f;

Adafruit_INA219 ina219(0x40);
CalibradorINA219 calibrador(PIN_BOTON_RESET, PIN_LED_ALERTA, SHUNT_OHMS, PIN_BUZZER);

constexpr unsigned long PERIODO_LECTURA_MS   = 100;  // evalua alertas (LED/buzzer)
constexpr unsigned long PERIODO_IMPRESION_MS = 600;  // imprime por serial
unsigned long tLectura   = 0;
unsigned long tImpresion = 0;

void setup() {
    Serial.begin(115200);
    delay(400);

    Wire.begin(PIN_SDA, PIN_SCL);

    // Si falla el sensor, el LED (GPIO 25) queda fijo y se reintenta cada 2 s
    while (!calibrador.iniciar(ina219, RANGO_800MA)) {
        delay(2000);
    }
}

void loop() {
    calibrador.actualizar(ina219);

    unsigned long ahora = millis();
    if (ahora - tLectura >= PERIODO_LECTURA_MS) {
        tLectura = ahora;

        LecturaCalibrada lec = calibrador.leer(ina219);

        if (ahora - tImpresion >= PERIODO_IMPRESION_MS) {
            tImpresion = ahora;

            Serial.printf("V_bus: %6.3f V | V_shunt: %6.2f mV | I_cruda: %7.2f mA | I_cal: %7.2f mA | P: %7.1f mW",
                          lec.voltajeBus_V,
                          lec.voltajeShunt_mV,
                          lec.corrienteCruda_mA,
                          lec.corrienteCalibrada_mA,
                          lec.potencia_mW);

            if (lec.hayAlerta) {
                Serial.print("  [ALERTA LED ON: ");
                Serial.print(lec.motivoAlerta);
                Serial.print("]");
            }
            Serial.println();
        }
    }
}