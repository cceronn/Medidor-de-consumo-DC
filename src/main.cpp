#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "Calibration.h"

// Pines del circuito
constexpr uint8_t PIN_BOTON_RESET = 33;
constexpr uint8_t PIN_LED_ALERTA  = 32;  // Si falla el sensor al iniciar, el LED en GPIO 32 queda encendido fijo como alerta
constexpr uint8_t PIN_SDA         = 21;
constexpr uint8_t PIN_SCL         = 22;

// Shunt R100 = 0.1 Ohmios alimentado a 10 V
constexpr float SHUNT_OHMS = 0.1f;

Adafruit_INA219 ina219(0x40);
CalibradorINA219 calibrador(PIN_BOTON_RESET, PIN_LED_ALERTA, SHUNT_OHMS);

unsigned long tAnterior = 0;
constexpr unsigned long PERIODO_IMPRESION_MS = 600;

void setup() {
    Serial.begin(115200);
    delay(400);

    Wire.begin(PIN_SDA, PIN_SCL);

    // Si falla el sensor al iniciar, el LED en GPIO 5 queda encendido fijo como alerta
    if (!calibrador.iniciar(ina219, RANGO_800MA)) {
        while (true) delay(1000);
    }
}

void loop() {
    calibrador.actualizar(ina219);

    if (millis() - tAnterior >= PERIODO_IMPRESION_MS) {
        tAnterior = millis();

        LecturaCalibrada lec = calibrador.leer(ina219);

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