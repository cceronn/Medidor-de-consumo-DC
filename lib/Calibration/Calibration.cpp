#include "Calibration.h"
#include <math.h>
#include <string.h>

namespace {

// Convierte texto a float validando; acepta coma o punto decimal
bool parsearNumero(String s, float &out) {
    s.trim();
    s.replace(',', '.');
    if (s.length() == 0) return false;
    char *fin = nullptr;
    float v = strtof(s.c_str(), &fin);
    if (*fin != '\0') return false;
    if (isnan(v) || isinf(v)) return false;
    out = v;
    return true;
}

bool finito(float x) { return !isnan(x) && !isinf(x); }

} // namespace

// ============================================================================
// Constructor / inicio
// ============================================================================
CalibradorINA219::CalibradorINA219(uint8_t pinBoton, uint8_t pinLed, float rShunt, uint8_t pinBuzzer) {
    _pinBoton = pinBoton;
    _pinLed = pinLed;
    _pinBuzzer = pinBuzzer;
    _rShunt = rShunt;

    _params.pendienteCorriente = 1.0f;
    _params.interceptoCorriente = 0.0f;
    _params.r2Corriente = NAN;
    _params.pendienteVoltaje = 1.0f;
    _params.interceptoVoltaje = 0.0f;
    _params.rangoPGA = RANGO_800MA;
    _params.calibrado = false;

    memset(_puntosI, 0, sizeof(_puntosI));
    memset(_puntosV, 0, sizeof(_puntosV));
    _numPuntosI = 0;
    _numPuntosV = 0;

    _pendienteI = false;
    _pendienteV = false;
    _capI = 0.0f;
    _capV = 0.0f;

    _estadoBoton = HIGH;
    _lecturaPrevia = HIGH;
    _tCambio = 0;
    _tPresionado = 0;
    _accionLarga = false;

    // Sistema nominal de 12 V (esquema Rev C). El umbral de corriente se fija segun el rango.
    _umbralCorrienteMax_mA = 780.0f;
    _umbralVoltajeMax_V = 13.5f;
    _toleranciaMaxPct = 1.0f;
    _alertaCalibracion = false;
    _motivoAlertaCalibracion = "";

    _vShuntFS_mV = 80.0f;
    _fullScale_mA = 800.0f;

    _estadoBuzzer = false;
    _tBuzzer = 0;
}

bool CalibradorINA219::iniciar(Adafruit_INA219 &ina, RangoCorrientePGA rangoInicial) {
    pinMode(_pinBoton, INPUT_PULLUP);
    pinMode(_pinLed, OUTPUT);
    digitalWrite(_pinLed, LOW);
    if (_pinBuzzer != SIN_PIN) {
        pinMode(_pinBuzzer, OUTPUT);
        digitalWrite(_pinBuzzer, LOW);
    }

    _estadoBoton = digitalRead(_pinBoton);
    _lecturaPrevia = _estadoBoton;
    _tCambio = millis();

    if (!ina.begin()) {
        digitalWrite(_pinLed, HIGH); // Sensor no detectado -> LED fijo
        Serial.println(F("[ALERTA CRITICA] INA219 no encontrado en I2C (0x40). Revisa SDA=21 y SCL=22."));
        return false;
    }

    cargarDeFlash();
    RangoCorrientePGA rangoActivo = (_params.calibrado && _params.rangoPGA <= RANGO_3200MA)
                                        ? (RangoCorrientePGA)_params.rangoPGA
                                        : rangoInicial;

    if (!configurarHardwareAltaPrecision(ina, rangoActivo)) {
        digitalWrite(_pinLed, HIGH);
        Serial.println(F("[ALERTA CRITICA] No se pudo escribir la configuracion en el INA219."));
        return false;
    }

    digitalWrite(_pinLed, LOW);
    imprimirMenuAyuda();
    imprimirReporteError();
    return true;
}

// ============================================================================
// Hardware
// ============================================================================
bool CalibradorINA219::configurarHardwareAltaPrecision(Adafruit_INA219 &ina, RangoCorrientePGA rango) {
    _params.rangoPGA = (uint8_t)rango;

    // Calibracion interna de la libreria (solo afecta getCurrent/getPower; nosotros
    // calculamos la corriente desde el voltaje del shunt, que no depende de ella).
    if (rango == RANGO_400MA) ina.setCalibration_16V_400mA();
    else                      ina.setCalibration_32V_2A();

    // Registro 0x00 (CONFIG):
    //   bit 13      BRNG = 1 -> rango de bus 32 V (LSB sigue siendo 4 mV; admite hasta 26 V)
    //   bits 12-11  PG       -> fondo de escala del shunt (40/80/160/320 mV)
    //   bits 10-7   BADC = 1111 -> promedio de 128 muestras
    //   bits 6-3    SADC = 1111 -> promedio de 128 muestras
    //   bits 2-0    MODE = 111  -> shunt y bus continuo
    uint16_t pg = ((uint16_t)rango) << 11;
    uint16_t configReg = 0x2000 | pg | 0x0780 | 0x0078 | 0x0007;

    Wire.beginTransmission(DIRECCION_INA219);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)((configReg >> 8) & 0xFF));
    Wire.write((uint8_t)(configReg & 0xFF));
    if (Wire.endTransmission() != 0) return false;

    // Fondo de escala y umbral de alerta derivados del shunt real
    _vShuntFS_mV = 40.0f * (float)(1 << (uint8_t)rango);
    _fullScale_mA = _vShuntFS_mV / _rShunt;
    _umbralCorrienteMax_mA = 0.975f * _fullScale_mA;

    delay(200); // 2 conversiones de 68 ms con promedio de 128 muestras
    return true;
}

float CalibradorINA219::leerCorrienteCruda(Adafruit_INA219 &ina) {
    // I = Vshunt / R (independiente del registro de calibracion de la libreria)
    return ina.getShuntVoltage_mV() / _rShunt;
}

bool CalibradorINA219::verificarConexionI2C() {
    Wire.beginTransmission(DIRECCION_INA219);
    return (Wire.endTransmission() == 0);
}

// ============================================================================
// Bucle de actualizacion: boton + serial
// ============================================================================
void CalibradorINA219::actualizar(Adafruit_INA219 &ina) {
    bool lectura = digitalRead(_pinBoton);
    unsigned long ahora = millis();

    if (lectura != _lecturaPrevia) {
        _lecturaPrevia = lectura;
        _tCambio = ahora;
    }

    // Antirrebote: el estado debe mantenerse 30 ms
    if ((ahora - _tCambio) >= 30 && lectura != _estadoBoton) {
        _estadoBoton = lectura;
        if (_estadoBoton == LOW) {
            _tPresionado = ahora;
            _accionLarga = false;
        } else {
            unsigned long duracion = ahora - _tPresionado;
            if (!_accionLarga && duracion < 2500) {
                capturarPuntoMuestra(ina, 100);
            }
        }
    }

    // Pulsacion larga (> 2.5 s): restablecer fabrica
    if (_estadoBoton == LOW && !_accionLarga && (ahora - _tPresionado) >= 2500) {
        _accionLarga = true;
        restablecerFabrica(ina);
    }

    procesarSerial(ina);
}

void CalibradorINA219::procesarSerial(Adafruit_INA219 &ina) {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (_bufferSerial.length() > 0) {
                String cmd = _bufferSerial;
                _bufferSerial = "";
                ejecutarComando(ina, cmd);
            }
        } else if (_bufferSerial.length() < 40) {
            _bufferSerial += c;
        }
    }
}

void CalibradorINA219::ejecutarComando(Adafruit_INA219 &ina, String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    float v = 0.0f;

    if (cmd == "CAP") {
        capturarPuntoMuestra(ina, 100);
    }
    else if (cmd.startsWith("MI:")) {
        if (parsearNumero(cmd.substring(3), v)) registrarMultimetroCorriente(v);
        else Serial.println(F("[AVISO] Valor invalido. Ejemplo: MI:148.65"));
    }
    else if (cmd.startsWith("MV:")) {
        if (parsearNumero(cmd.substring(3), v)) registrarMultimetroVoltaje(v);
        else Serial.println(F("[AVISO] Valor invalido. Ejemplo: MV:10.012"));
    }
    else if (cmd == "CERO") {
        calibrarCeroRapido(ina, 150);
    }
    else if (cmd.startsWith("RANGO:")) {
        int r = cmd.substring(6).toInt();
        if (r == 400)       cambiarRangoPGA(ina, RANGO_400MA);
        else if (r == 800)  cambiarRangoPGA(ina, RANGO_800MA);
        else if (r == 1600) cambiarRangoPGA(ina, RANGO_1600MA);
        else if (r == 3200) cambiarRangoPGA(ina, RANGO_3200MA);
        else Serial.println(F("[AVISO] Rangos validos: RANGO:400, RANGO:800, RANGO:1600, RANGO:3200"));
    }
    else if (cmd.startsWith("AL_I:")) {
        if (parsearNumero(cmd.substring(5), v)) configurarUmbralCorriente(v);
        else Serial.println(F("[AVISO] Valor invalido. Ejemplo: AL_I:500"));
    }
    else if (cmd.startsWith("AL_V:")) {
        if (parsearNumero(cmd.substring(5), v)) configurarUmbralVoltaje(v);
        else Serial.println(F("[AVISO] Valor invalido. Ejemplo: AL_V:13.5"));
    }
    else if (cmd.startsWith("TOL:")) {
        if (parsearNumero(cmd.substring(4), v)) configurarToleranciaMaxima(v);
        else Serial.println(F("[AVISO] Valor invalido. Ejemplo: TOL:0.5"));
    }
    else if (cmd == "ACK")     limpiarAlertaCalibracion();
    else if (cmd == "LIMPIAR") borrarPuntosTemporales();
    else if (cmd == "RESET")   restablecerFabrica(ina);
    else if (cmd == "INFO")    imprimirReporteError();
    else if (cmd == "AYUDA")   imprimirMenuAyuda();
    else Serial.println(F("[AVISO] Comando no reconocido. Envia AYUDA."));
}

// ============================================================================
// Captura y registro de puntos
// ============================================================================
void CalibradorINA219::lanzarAlertaCalibracion(const char* motivo, bool imprimir) {
    _alertaCalibracion = true;
    _motivoAlertaCalibracion = motivo;
    actualizarEstadoLed(true);
    if (imprimir) {
        Serial.print(F("[ALERTA] "));
        Serial.println(motivo);
    }
}

void CalibradorINA219::capturarPuntoMuestra(Adafruit_INA219 &ina, uint16_t muestras) {
    if (!verificarConexionI2C()) {
        lanzarAlertaCalibracion("Sin comunicacion I2C con INA219");
        return;
    }
    if (muestras < 2) muestras = 2;

    Serial.print(F("\n[CAPTURA] Promediando "));
    Serial.print(muestras);
    Serial.println(F(" muestras... Manten la carga estable."));

    double sumaI = 0.0, sumaI2 = 0.0, sumaV = 0.0;
    for (uint16_t i = 0; i < muestras; i++) {
        double c = leerCorrienteCruda(ina);
        sumaI  += c;
        sumaI2 += c * c;
        sumaV  += ina.getBusVoltage_V();
        delay(15);
    }

    double mediaI = sumaI / muestras;
    double var = (sumaI2 / muestras) - (mediaI * mediaI);
    if (var < 0.0) var = 0.0;
    double sigma = sqrt(var);

    // Un valor pegado al fondo de escala es el limite del PGA, no una medida
    if (fabs(mediaI) >= 0.99 * _fullScale_mA) {
        _pendienteI = false;
        _pendienteV = false;
        lanzarAlertaCalibracion("Rango saturado: la carga excede el fondo de escala. Captura descartada");
        return;
    }

    _capI = (float)mediaI;
    _capV = (float)(sumaV / muestras);
    _pendienteI = true;
    _pendienteV = true;
    limpiarAlertaCalibracion();

    Serial.println(F("=========================================================="));
    Serial.print(F("  LECTURA CONGELADA -> I_cruda = "));
    Serial.print(_capI, 3);
    Serial.print(F(" mA (sigma = "));
    Serial.print(sigma, 3);
    Serial.print(F(") | V_crudo = "));
    Serial.print(_capV, 4);
    Serial.println(F(" V"));
    if (sigma > fmax(0.3, 0.005 * fabs(mediaI))) {
        Serial.println(F("  [AVISO] La corriente fluctua mucho durante la captura; el punto puede ser poco fiable."));
    }
    Serial.println(F("  Ingresa la lectura del multimetro patron (puedes enviar ambas):"));
    Serial.println(F("    -> MI:<valor_mA>   (Ej: MI:148.65 o MI:0)"));
    Serial.println(F("    -> MV:<valor_V>    (Ej: MV:10.012)"));
    Serial.println(F("==========================================================\n"));
}

void CalibradorINA219::registrarMultimetroCorriente(float corrienteMultimetro_mA) {
    if (!_pendienteI) {
        lanzarAlertaCalibracion("Registro MI sin captura previa (boton GPIO 33 o 'CAP')");
        return;
    }
    if (_numPuntosI >= MAX_PUNTOS) {
        lanzarAlertaCalibracion("Tabla de corriente llena (max 10). Envia 'LIMPIAR'");
        return;
    }
    agregarPuntoCorriente(_capI, corrienteMultimetro_mA);
    _pendienteI = false;
}

void CalibradorINA219::registrarMultimetroVoltaje(float voltajeMultimetro_V) {
    if (!_pendienteV) {
        lanzarAlertaCalibracion("Registro MV sin captura previa (boton GPIO 33 o 'CAP')");
        return;
    }
    if (_numPuntosV >= MAX_PUNTOS) {
        lanzarAlertaCalibracion("Tabla de voltaje llena (max 10). Envia 'LIMPIAR'");
        return;
    }
    // Filtro anti-errata: el multimetro no puede diferir >2x del sensor
    if (_capV > 0.1f) {
        float ratio = voltajeMultimetro_V / _capV;
        if (ratio < 0.5f || ratio > 2.0f) {
            lanzarAlertaCalibracion("Valor MV incoherente con el sensor. Punto rechazado");
            return;
        }
    }
    agregarPuntoVoltaje(_capV, voltajeMultimetro_V);
    _pendienteV = false;
}

void CalibradorINA219::agregarPuntoCorriente(float crudo, float real) {
    _puntosI[_numPuntosI].lecturaSensor = crudo;
    _puntosI[_numPuntosI].lecturaMultimetro = real;
    _numPuntosI++;

    Serial.print(F("[OK] Punto I #"));
    Serial.print(_numPuntosI);
    Serial.print(F(": Sensor = "));
    Serial.print(crudo, 3);
    Serial.print(F(" mA  <--->  Patron = "));
    Serial.print(real, 3);
    Serial.println(F(" mA"));

    if (_numPuntosI == 1) {
        _params.pendienteCorriente = 1.0f;
        _params.interceptoCorriente = real - crudo;
        _params.r2Corriente = NAN;
        Serial.println(F("  -> 1 punto: solo se ajusta offset. Agrega puntos con cargas distintas para la pendiente."));
    } else {
        // Separacion minima: 5 % del fondo de escala; si no, solo offset (m = 1)
        bool ok = calcularRegresionLineal(_puntosI, _numPuntosI, 0.05f * _fullScale_mA, false,
                                          _params.pendienteCorriente,
                                          _params.interceptoCorriente,
                                          _params.r2Corriente);
        if (!ok) {
            Serial.println(F("  [AVISO] Puntos muy juntos (< 5% del rango): solo se ajusto offset (m = 1)."));
            Serial.println(F("          Agrega puntos de corriente mas separados para ajustar la pendiente."));
        }
    }

    _params.calibrado = true;

    if (hayPuntosFueraDeTolerancia()) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Hay puntos fuera de tolerancia (%.2f%% o %.1f mA)", _toleranciaMaxPct, PISO_ERROR_mA);
        lanzarAlertaCalibracion(msg);
        Serial.println(F("  Revisa la tabla y envia 'LIMPIAR' si hubo un error de lectura."));
    } else {
        limpiarAlertaCalibracion();
    }

    guardarEnFlash();
    imprimirReporteError();
}

void CalibradorINA219::agregarPuntoVoltaje(float crudo, float real) {
    _puntosV[_numPuntosV].lecturaSensor = crudo;
    _puntosV[_numPuntosV].lecturaMultimetro = real;
    _numPuntosV++;

    if (_numPuntosV == 1) {
        if (crudo > 0.1f) {
            _params.pendienteVoltaje = real / crudo;
            _params.interceptoVoltaje = 0.0f;
        }
    } else {
        float r2V = NAN;
        // Separacion minima 1 V; si no, solo ganancia (b = 0)
        bool ok = calcularRegresionLineal(_puntosV, _numPuntosV, 1.0f, true,
                                          _params.pendienteVoltaje,
                                          _params.interceptoVoltaje,
                                          r2V);
        if (!ok) {
            Serial.println(F("  [AVISO] Puntos de voltaje muy juntos (< 1 V): solo se ajusto ganancia (b = 0)."));
            Serial.println(F("          Para ajustar offset toma otro punto con una tension distinta."));
        }
    }

    _params.calibrado = true;
    limpiarAlertaCalibracion();
    guardarEnFlash();

    Serial.print(F("[OK] Punto V #"));
    Serial.print(_numPuntosV);
    Serial.print(F(" -> V_real = "));
    Serial.print(_params.pendienteVoltaje, 6);
    Serial.print(F(" * V_sensor + ("));
    Serial.print(_params.interceptoVoltaje, 4);
    Serial.println(F(" V)\n"));
}

// Regresion lineal y = m*x + b. Devuelve false si los puntos estan demasiado
// juntos (en ese caso usa un ajuste degenerado: solo offset o solo ganancia).
bool CalibradorINA219::calcularRegresionLineal(const PuntoCalibracion* puntos, uint8_t n,
                                               float separacionMin, bool fallbackGanancia,
                                               float &m, float &b, float &r2) {
    double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0, sumY2 = 0.0;
    double xmin = puntos[0].lecturaSensor, xmax = xmin;

    for (uint8_t i = 0; i < n; i++) {
        double x = puntos[i].lecturaSensor;
        double y = puntos[i].lecturaMultimetro;
        sumX += x;  sumY += y;
        sumXY += x * y;  sumX2 += x * x;  sumY2 += y * y;
        if (x < xmin) xmin = x;
        if (x > xmax) xmax = x;
    }

    double denom = ((double)n * sumX2) - (sumX * sumX);
    if ((xmax - xmin) < separacionMin || fabs(denom) < 1e-12) {
        if (fallbackGanancia && fabs(sumX) > 1e-9) {
            m = (float)(sumY / sumX);
            b = 0.0f;
        } else {
            m = 1.0f;
            b = (float)((sumY - sumX) / n);
        }
        r2 = NAN;
        return false;
    }

    double slope = (((double)n * sumXY) - (sumX * sumY)) / denom;
    double intercept = (sumY - slope * sumX) / n;

    double numR = ((double)n * sumXY) - (sumX * sumY);
    double denR = sqrt(denom * (((double)n * sumY2) - (sumY * sumY)));
    double r = (denR > 1e-12) ? (numR / denR) : 0.0;

    m = (float)slope;
    b = (float)intercept;
    r2 = (float)(r * r);
    return true;
}

// Error admitido = max(tolerancia% * valor, piso de 3 LSB). Evita falsas alarmas a baja corriente.
bool CalibradorINA219::puntoFueraDeTolerancia(float crudo, float real) const {
    float cal = (crudo * _params.pendienteCorriente) + _params.interceptoCorriente;
    float limite = fmaxf((_toleranciaMaxPct / 100.0f) * fabsf(real), PISO_ERROR_mA);
    return fabsf(cal - real) > limite;
}

bool CalibradorINA219::hayPuntosFueraDeTolerancia() const {
    for (uint8_t i = 0; i < _numPuntosI; i++) {
        if (puntoFueraDeTolerancia(_puntosI[i].lecturaSensor, _puntosI[i].lecturaMultimetro)) return true;
    }
    return false;
}

// ============================================================================
// Cero, rango, borrado
// ============================================================================
void CalibradorINA219::calibrarCeroRapido(Adafruit_INA219 &ina, uint16_t muestras) {
    if (!verificarConexionI2C()) {
        lanzarAlertaCalibracion("Sin comunicacion I2C con INA219");
        return;
    }
    if (_numPuntosI >= MAX_PUNTOS) {
        lanzarAlertaCalibracion("Tabla de corriente llena (max 10). Envia 'LIMPIAR'");
        return;
    }

    Serial.println(F("\n[CERO] Midiendo offset en vacio (carga desconectada)..."));
    double suma = 0.0;
    for (uint16_t i = 0; i < muestras; i++) {
        suma += leerCorrienteCruda(ina);
        delay(15);
    }
    float crudoCero = (float)(suma / muestras);

    if (fabsf(crudoCero) > 2.0f) {
        lanzarAlertaCalibracion("CERO rechazado: |I| > 2 mA, la carga no parece desconectada");
        return;
    }

    // El cero se guarda como un punto mas (sensor, 0 mA) para que la regresion lo incluya
    agregarPuntoCorriente(crudoCero, 0.0f);
}

void CalibradorINA219::cambiarRangoPGA(Adafruit_INA219 &ina, RangoCorrientePGA nuevoRango) {
    if (!configurarHardwareAltaPrecision(ina, nuevoRango)) {
        lanzarAlertaCalibracion("No se pudo cambiar el rango (fallo I2C)");
        return;
    }
    borrarPuntosTemporales();
    Serial.print(F("[CONFIG] Rango activo: "));
    Serial.print(_fullScale_mA, 0);
    Serial.print(F(" mA. Umbral alerta corriente: "));
    Serial.print(_umbralCorrienteMax_mA, 1);
    Serial.println(F(" mA. Recalibra con puntos nuevos."));
}

void CalibradorINA219::borrarPuntosTemporales() {
    _numPuntosI = 0;
    _numPuntosV = 0;
    memset(_puntosI, 0, sizeof(_puntosI));
    memset(_puntosV, 0, sizeof(_puntosV));
    _pendienteI = false;
    _pendienteV = false;
    limpiarAlertaCalibracion();
    guardarEnFlash();
    Serial.println(F("[INFO] Tabla de puntos borrada (la ecuacion vigente se conserva; usa RESET para volver a m=1, b=0)."));
}

void CalibradorINA219::restablecerFabrica(Adafruit_INA219 &ina) {
    (void)ina;
    _params.pendienteCorriente = 1.0f;
    _params.interceptoCorriente = 0.0f;
    _params.r2Corriente = NAN;
    _params.pendienteVoltaje = 1.0f;
    _params.interceptoVoltaje = 0.0f;
    _params.calibrado = false;
    borrarPuntosTemporales(); // tambien guarda en flash
    Serial.println(F("\n[RESET] Calibracion restablecida a valores de fabrica (m=1, b=0).\n"));
}

void CalibradorINA219::limpiarAlertaCalibracion() {
    _alertaCalibracion = false;
    _motivoAlertaCalibracion = "";
    actualizarEstadoLed(false);
}

// ============================================================================
// Umbrales
// ============================================================================
void CalibradorINA219::configurarUmbralCorriente(float max_mA) {
    if (max_mA > 0.0f) {
        if (max_mA > _fullScale_mA) {
            max_mA = _fullScale_mA;
            Serial.println(F("[AVISO] Umbral limitado al fondo de escala del rango activo."));
        }
        _umbralCorrienteMax_mA = max_mA;
        Serial.print(F("[CONFIG] Nuevo umbral de alerta de corriente: "));
        Serial.print(_umbralCorrienteMax_mA, 2);
        Serial.println(F(" mA"));
    }
}

void CalibradorINA219::configurarUmbralVoltaje(float max_V) {
    if (max_V > 0.0f) {
        _umbralVoltajeMax_V = max_V;
        Serial.print(F("[CONFIG] Nuevo umbral de alerta de voltaje: "));
        Serial.print(_umbralVoltajeMax_V, 2);
        Serial.println(F(" V"));
    }
}

void CalibradorINA219::configurarToleranciaMaxima(float maxPct) {
    if (maxPct > 0.0f) {
        _toleranciaMaxPct = maxPct;
        Serial.print(F("[CONFIG] Nueva tolerancia maxima permitida: "));
        Serial.print(_toleranciaMaxPct, 3);
        Serial.println(F(" %"));
    }
}

// ============================================================================
// LED / buzzer
// ============================================================================
void CalibradorINA219::actualizarEstadoLed(bool alertaTiempoReal) {
    bool encender = alertaTiempoReal || _alertaCalibracion;
    digitalWrite(_pinLed, encender ? HIGH : LOW);
}

// Pitido corto (100 ms) cada ~1 s mientras haya alerta ELECTRICA
void CalibradorINA219::actualizarBuzzer(bool activo) {
    if (_pinBuzzer == SIN_PIN) return;
    if (!activo) {
        digitalWrite(_pinBuzzer, LOW);
        _estadoBuzzer = false;
        return;
    }
    unsigned long ahora = millis();
    unsigned long espera = _estadoBuzzer ? 100UL : 900UL;
    if (ahora - _tBuzzer >= espera) {
        _estadoBuzzer = !_estadoBuzzer;
        _tBuzzer = ahora;
        digitalWrite(_pinBuzzer, _estadoBuzzer ? HIGH : LOW);
    }
}

// ============================================================================
// Lectura
// ============================================================================
LecturaCalibrada CalibradorINA219::leer(Adafruit_INA219 &ina) {
    LecturaCalibrada d;
    d.voltajeBus_V = 0.0f;
    d.voltajeBusCrudo_V = 0.0f;
    d.voltajeShunt_mV = 0.0f;
    d.corrienteCruda_mA = 0.0f;
    d.corrienteCalibrada_mA = 0.0f;
    d.potencia_mW = 0.0f;
    d.hayAlerta = false;
    d.motivoAlerta = "";

    if (!verificarConexionI2C()) {
        d.hayAlerta = true;
        d.motivoAlerta = "DESCONEXION I2C INA219";
        actualizarEstadoLed(true);
        actualizarBuzzer(true);
        return d;
    }

    d.voltajeBusCrudo_V = ina.getBusVoltage_V();
    d.voltajeShunt_mV   = ina.getShuntVoltage_mV();
    d.corrienteCruda_mA = d.voltajeShunt_mV / _rShunt;

    d.voltajeBus_V = (d.voltajeBusCrudo_V * _params.pendienteVoltaje) + _params.interceptoVoltaje;
    d.corrienteCalibrada_mA = (d.corrienteCruda_mA * _params.pendienteCorriente) + _params.interceptoCorriente;

    if (fabsf(d.corrienteCalibrada_mA) < ZONA_MUERTA_mA) {
        d.corrienteCalibrada_mA = 0.0f;
    }

    d.potencia_mW = d.voltajeBus_V * d.corrienteCalibrada_mA;

    // Alertas electricas en tiempo real (por prioridad)
    bool alertaElectrica = false;
    if (fabsf(d.voltajeShunt_mV) >= 0.995f * _vShuntFS_mV) {
        alertaElectrica = true;
        d.motivoAlerta = "SATURACION PGA (Aumenta RANGO)";
    } else if (fabsf(d.corrienteCalibrada_mA) > _umbralCorrienteMax_mA) {
        alertaElectrica = true;
        d.motivoAlerta = "SOBRECORRIENTE (> " + String(_umbralCorrienteMax_mA, 0) + " mA)";
    } else if (d.voltajeBus_V > _umbralVoltajeMax_V) {
        alertaElectrica = true;
        d.motivoAlerta = "SOBREVOLTAJE (> " + String(_umbralVoltajeMax_V, 1) + " V)";
    }

    if (alertaElectrica) {
        d.hayAlerta = true;
    } else if (_alertaCalibracion) {
        d.hayAlerta = true;
        d.motivoAlerta = _motivoAlertaCalibracion;
    }

    actualizarEstadoLed(d.hayAlerta);
    actualizarBuzzer(alertaElectrica);
    return d;
}

// ============================================================================
// Reportes
// ============================================================================
void CalibradorINA219::imprimirReporteError() {
    Serial.println(F("\n================ REPORTE DE CALIBRACION Y TOLERANCIA ================"));
    Serial.print(F("Estado:        "));
    Serial.println(_params.calibrado ? F("CALIBRADO (guardado en flash ESP32)") : F("SIN CALIBRAR"));
    Serial.printf("Rango PGA:     fondo de escala = %.0f mA (+/- %.0f mV)\n", _fullScale_mA, _vShuntFS_mV);
    Serial.print(F("Ecuacion I:    I_real = "));
    Serial.print(_params.pendienteCorriente, 6);
    Serial.print(F(" * I_sensor + ("));
    Serial.print(_params.interceptoCorriente, 4);
    Serial.println(F(" mA)"));
    Serial.print(F("Linealidad:    R^2 = "));
    if (isnan(_params.r2Corriente)) Serial.println(F("n/d (pocos puntos o muy juntos)"));
    else                            Serial.println(_params.r2Corriente, 6);
    Serial.print(F("Ecuacion V:    V_real = "));
    Serial.print(_params.pendienteVoltaje, 6);
    Serial.print(F(" * V_sensor + ("));
    Serial.print(_params.interceptoVoltaje, 4);
    Serial.println(F(" V)"));
    Serial.printf("Umbrales LED:  I_max = %.1f mA | V_max = %.2f V | Tolerancia = %.2f%% (piso %.1f mA)\n",
                  _umbralCorrienteMax_mA, _umbralVoltajeMax_V, _toleranciaMaxPct, PISO_ERROR_mA);

    if (_numPuntosI > 0) {
        Serial.println(F("\n--- Verificacion vs multimetro patron (Corriente) ---"));
        Serial.println(F("Pt | Multimetro (mA) | Crudo (mA) | Calibrado (mA) | Error Abs | Error % | Estado"));
        for (uint8_t i = 0; i < _numPuntosI; i++) {
            float mult = _puntosI[i].lecturaMultimetro;
            float crud = _puntosI[i].lecturaSensor;
            float cal  = (crud * _params.pendienteCorriente) + _params.interceptoCorriente;
            float errAbs = cal - mult;
            float errPct = (fabsf(mult) > 0.5f) ? (fabsf(errAbs) / fabsf(mult)) * 100.0f : 0.0f;
            Serial.printf(" %d | %15.3f | %10.3f | %14.3f | %+8.3f | %6.3f%% | %s\n",
                          i + 1, mult, crud, cal, errAbs, errPct,
                          puntoFueraDeTolerancia(crud, mult) ? "FUERA" : "OK");
        }
    }

    if (_numPuntosV > 0) {
        Serial.println(F("\n--- Verificacion vs multimetro patron (Voltaje) ---"));
        Serial.println(F("Pt | Multimetro (V) | Crudo (V) | Calibrado (V) | Error Abs (mV)"));
        for (uint8_t i = 0; i < _numPuntosV; i++) {
            float mult = _puntosV[i].lecturaMultimetro;
            float crud = _puntosV[i].lecturaSensor;
            float cal  = (crud * _params.pendienteVoltaje) + _params.interceptoVoltaje;
            Serial.printf(" %d | %14.4f | %9.4f | %13.4f | %+9.1f\n",
                          i + 1, mult, crud, cal, (cal - mult) * 1000.0f);
        }
    }
    Serial.println(F("=====================================================================\n"));
}

void CalibradorINA219::imprimirMenuAyuda() {
    Serial.println(F("\n--- COMANDOS DE CALIBRACION Y ALERTAS ---"));
    Serial.println(F("  Boton GPIO 33 (corto) o 'CAP'  -> Congela y promedia 100 muestras (I y V)"));
    Serial.println(F("  MI:125.40                      -> Registra lectura del multimetro en mA"));
    Serial.println(F("  MV:12.015                      -> Registra lectura del multimetro en V"));
    Serial.println(F("                                    (tras un CAP puedes enviar MI y MV)"));
    Serial.println(F("  CERO                           -> Punto (sensor, 0 mA) con carga desconectada"));
    Serial.println(F("  RANGO:400 / 800 / 1600 / 3200  -> Cambia fondo de escala del PGA (borra puntos)"));
    Serial.println(F("  AL_I:500  /  AL_V:13.5         -> Umbrales que activan LED y buzzer"));
    Serial.println(F("  TOL:0.5                        -> Tolerancia max (%) para alerta de calibracion"));
    Serial.println(F("  ACK                            -> Apaga alerta de calibracion latente"));
    Serial.println(F("  LIMPIAR                        -> Borra la tabla de puntos"));
    Serial.println(F("  INFO                           -> Ecuaciones y tabla de error"));
    Serial.println(F("  Boton GPIO 33 (>2.5 s) o RESET -> Borra calibracion de flash"));
    Serial.println(F("  Tip: puntos de corriente separados >= 5% del rango para ajustar pendiente.\n"));
}

// ============================================================================
// Flash (parametros + puntos)
// ============================================================================
void CalibradorINA219::guardarEnFlash() {
    _prefs.begin("ina219_metro", false);
    _prefs.putFloat("m_i", _params.pendienteCorriente);
    _prefs.putFloat("b_i", _params.interceptoCorriente);
    _prefs.putFloat("r2_i", _params.r2Corriente);
    _prefs.putFloat("m_v", _params.pendienteVoltaje);
    _prefs.putFloat("b_v", _params.interceptoVoltaje);
    _prefs.putUChar("pga", _params.rangoPGA);
    _prefs.putBool("cal", _params.calibrado);
    _prefs.putUChar("n_i", _numPuntosI);
    _prefs.putUChar("n_v", _numPuntosV);
    _prefs.putBytes("pts_i", _puntosI, sizeof(_puntosI));
    _prefs.putBytes("pts_v", _puntosV, sizeof(_puntosV));
    _prefs.end();
}

void CalibradorINA219::cargarDeFlash() {
    _prefs.begin("ina219_metro", false); // lectura/escritura: crea el namespace si no existe
    _params.pendienteCorriente  = _prefs.getFloat("m_i", 1.0f);
    _params.interceptoCorriente = _prefs.getFloat("b_i", 0.0f);
    _params.r2Corriente         = _prefs.getFloat("r2_i", NAN);
    _params.pendienteVoltaje    = _prefs.getFloat("m_v", 1.0f);
    _params.interceptoVoltaje   = _prefs.getFloat("b_v", 0.0f);
    _params.rangoPGA            = _prefs.getUChar("pga", RANGO_800MA);
    _params.calibrado           = _prefs.getBool("cal", false);

    _numPuntosI = _prefs.getUChar("n_i", 0);
    _numPuntosV = _prefs.getUChar("n_v", 0);
    if (_numPuntosI > MAX_PUNTOS ||
        _prefs.getBytes("pts_i", _puntosI, sizeof(_puntosI)) != sizeof(_puntosI)) {
        _numPuntosI = 0;
        memset(_puntosI, 0, sizeof(_puntosI));
    }
    if (_numPuntosV > MAX_PUNTOS ||
        _prefs.getBytes("pts_v", _puntosV, sizeof(_puntosV)) != sizeof(_puntosV)) {
        _numPuntosV = 0;
        memset(_puntosV, 0, sizeof(_puntosV));
    }
    _prefs.end();

    // Validacion: si hay valores corruptos, volver a fabrica
    if (!finito(_params.pendienteCorriente) || !finito(_params.interceptoCorriente) ||
        !finito(_params.pendienteVoltaje)   || !finito(_params.interceptoVoltaje)) {
        _params.pendienteCorriente = 1.0f;
        _params.interceptoCorriente = 0.0f;
        _params.pendienteVoltaje = 1.0f;
        _params.interceptoVoltaje = 0.0f;
        _params.r2Corriente = NAN;
        _params.calibrado = false;
    }
}