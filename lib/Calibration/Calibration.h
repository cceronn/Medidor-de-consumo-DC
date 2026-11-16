#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Preferences.h>

// El PGA solo define el FONDO DE ESCALA del shunt (no cambia la resolucion):
//   LSB de shunt = 10 uV -> con 0.1 ohm = 0.1 mA en TODOS los rangos.
enum RangoCorrientePGA : uint8_t {
    RANGO_400MA  = 0, // PGA +/- 40 mV  -> 400 mA con 0.1 ohm
    RANGO_800MA  = 1, // PGA +/- 80 mV  -> 800 mA
    RANGO_1600MA = 2, // PGA +/- 160 mV -> 1600 mA
    RANGO_3200MA = 3  // PGA +/- 320 mV -> 3200 mA
};

struct PuntoCalibracion {
    float lecturaSensor;
    float lecturaMultimetro;
};

struct ParametrosCalibracion {
    float pendienteCorriente;
    float interceptoCorriente;
    float r2Corriente;          // NAN = no definido
    float pendienteVoltaje;
    float interceptoVoltaje;
    uint8_t rangoPGA;
    bool calibrado;
};

struct LecturaCalibrada {
    float voltajeBus_V;         // Tension en el lado carga (VIN-), calibrada
    float voltajeBusCrudo_V;
    float voltajeShunt_mV;
    float corrienteCruda_mA;
    float corrienteCalibrada_mA;
    float potencia_mW;
    bool hayAlerta;
    String motivoAlerta;
};

class CalibradorINA219 {
private:
    static constexpr uint8_t MAX_PUNTOS = 10;
    static constexpr uint8_t DIRECCION_INA219 = 0x40;
    static constexpr uint8_t SIN_PIN = 255;
    static constexpr float ZONA_MUERTA_mA = 0.15f;  // ~1.5 LSB
    static constexpr float PISO_ERROR_mA  = 0.3f;   // error minimo aceptable (3 LSB)

    uint8_t _pinBoton;
    uint8_t _pinLed;
    uint8_t _pinBuzzer;
    float _rShunt;

    ParametrosCalibracion _params;
    Preferences _prefs;

    // Puntos de calibracion multipunto (se guardan en flash)
    PuntoCalibracion _puntosI[MAX_PUNTOS];
    uint8_t _numPuntosI;
    PuntoCalibracion _puntosV[MAX_PUNTOS];
    uint8_t _numPuntosV;

    // Captura congelada: corriente y voltaje se consumen por separado
    bool _pendienteI;
    bool _pendienteV;
    float _capI;
    float _capV;

    // Boton fisico con antirrebote
    bool _estadoBoton;       // estado estable
    bool _lecturaPrevia;     // ultima lectura cruda
    unsigned long _tCambio;
    unsigned long _tPresionado;
    bool _accionLarga;

    // Buffer de linea para el Monitor Serial (no bloqueante)
    String _bufferSerial;

    // Alertas
    float _umbralCorrienteMax_mA;
    float _umbralVoltajeMax_V;
    float _toleranciaMaxPct;
    bool _alertaCalibracion;
    String _motivoAlertaCalibracion;

    // Rango activo
    float _vShuntFS_mV;
    float _fullScale_mA;

    // Buzzer intermitente
    bool _estadoBuzzer;
    unsigned long _tBuzzer;

    bool configurarHardwareAltaPrecision(Adafruit_INA219 &ina, RangoCorrientePGA rango);
    float leerCorrienteCruda(Adafruit_INA219 &ina);
    bool calcularRegresionLineal(const PuntoCalibracion* puntos, uint8_t n,
                                 float separacionMin, bool fallbackGanancia,
                                 float &m, float &b, float &r2);
    void agregarPuntoCorriente(float crudo, float real);
    void agregarPuntoVoltaje(float crudo, float real);
    bool puntoFueraDeTolerancia(float crudo, float real) const;
    bool hayPuntosFueraDeTolerancia() const;
    void lanzarAlertaCalibracion(const char* motivo, bool imprimir = true);
    void actualizarEstadoLed(bool alertaTiempoReal);
    void actualizarBuzzer(bool activo);
    bool verificarConexionI2C();
    void guardarEnFlash();
    void cargarDeFlash();
    void procesarSerial(Adafruit_INA219 &ina);
    void ejecutarComando(Adafruit_INA219 &ina, String cmd);

public:
    CalibradorINA219(uint8_t pinBoton, uint8_t pinLed, float rShunt = 0.1f, uint8_t pinBuzzer = 255);

    bool iniciar(Adafruit_INA219 &ina, RangoCorrientePGA rangoInicial = RANGO_800MA);
    void actualizar(Adafruit_INA219 &ina);

    // Calibracion asistida por multimetro
    void capturarPuntoMuestra(Adafruit_INA219 &ina, uint16_t muestras = 100);
    void registrarMultimetroCorriente(float corrienteMultimetro_mA);
    void registrarMultimetroVoltaje(float voltajeMultimetro_V);
    void calibrarCeroRapido(Adafruit_INA219 &ina, uint16_t muestras = 150);
    void cambiarRangoPGA(Adafruit_INA219 &ina, RangoCorrientePGA nuevoRango);
    void borrarPuntosTemporales();
    void restablecerFabrica(Adafruit_INA219 &ina);
    void limpiarAlertaCalibracion();

    // Umbrales de alerta
    void configurarUmbralCorriente(float max_mA);
    void configurarUmbralVoltaje(float max_V);
    void configurarToleranciaMaxima(float maxPct);

    // Lectura y reportes
    LecturaCalibrada leer(Adafruit_INA219 &ina);
    void imprimirReporteError();
    void imprimirMenuAyuda();
};

#endif