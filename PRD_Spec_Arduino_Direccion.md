Product Requirements Document & Spec — Arduino Dirección

**Subsistema:** Módulo de Dirección (Sensórica) — micro-tuneladora B.I.C.H.O. (UDD Tunnel Lab)

**Revisión:** 2.1 (Final - Sincronizado a Backend) · **Fase:** MVP (Pruebas físicas de laboratorio y lazo cerrado)

**Hardware:** Arduino Mega 2560 + Ethernet Shield W5500

**Alimentación:** Bus DC de bajo voltaje aislado (PoE con splitter a 9-12V)

**Lenguaje:** C++ / Arduino Framework (Clean Architecture)

**Red:** IP estática `192.168.1.10` (Tráfico saliente UDP Push)

---

## 1. Visión y Regla Inquebrantable de Arquitectura

Este nodo microcontrolador actúa exclusivamente como el sistema nervioso periférico del módulo de dirección de la tuneladora (mecanismo de cuñas rotatorias, ±10°). Adquiere datos físicos ininterrumpidamente y los transmite al Servidor Central (Python).

> **REGLA DE ACERO (Solo Lectura):** Este dispositivo tiene TERMINANTEMENTE PROHIBIDO ejecutar lógica de control, evaluar interlocks o enviar comandos eléctricos. El control motriz de 480 V recae exclusivamente en el PLC SEW independiente. Toda decisión de aborto y validación cruzada vive en el Servidor Central.

---

## 2. Capa Física y Hardening Industrial EMI

El entorno de 480 V exige protecciones severas contra ruido eléctrico e inducción.

| ID | Señal | Interfaz / Requisito Eléctrico |

|---|---|---|

| `DIR-01` | IMU (Roll, Pitch, Heading) | **I2C diferencial.** Obligatorio transceptor PCA9615 en ambos extremos sobre par trenzado. Reloj a 100 kHz. |

| `DIR-02` | Encoder absoluto — Anillo A | **Externo.** Montado mecánicamente en la cuña, independiente del motor SEW. |

| `DIR-03` | Encoder absoluto — Anillo B | **Externo.** Montado mecánicamente en la cuña, independiente del motor SEW. |

| `DIR-04` | Fin de carrera +10° (Tope A) | **Normalmente Cerrado (NC).** Entrada optoacoplada a 5V. |

| `DIR-05` | Fin de carrera −10° (Tope B) | **Normalmente Cerrado (NC).** Entrada optoacoplada a 5V. |

| `DIR-06` | Inclinómetro doble eje | Sensor de redundancia física. Lectura pasiva. |

*Nota de Seguridad en Fines de Carrera (NC):* El cableado debe garantizar que un circuito abierto (cable cortado, ficha desconectada) se lea como `0` lógico en el pin (falla/tope activo), desencadenando una condición segura.

---

## 3. Capacidades de Software (Capabilities)

### CAP-1: Bucle de Adquisición No Bloqueante (50 Hz)

El código evalúa `millis()` para muestrear los sensores físicos a exactamente **50 Hz** (cada 20 ms).

* **Prohibición Estricta:** Cero uso de `delay()`.
* **Debounce y Latch:** Los fines de carrera deben filtrarse por software requiriendo un estado estable de mínimo **50 ms** (3 lecturas consecutivas a 50Hz) antes de validar un cambio. Todo tope detectado `0` lógico) debe quedar retenido (latch) en memoria hasta ser transmitido por red para no perder impactos rápidos.
* **Timeouts de Bus:** Toda lectura (I2C/SPI) debe tener un timeout acotado. Un sensor no puede congelar el `loop()`.

### CAP-2: Transmisión Binaria y Resiliencia UDP (10 Hz)

Los datos se empujan proactivamente vía socket UDP a **10 Hz**, sincronizándose con el bucle maestro del Servidor Python.

* **Bloqueo de Ethernet Prevenido:** La función `endPacket()` del W5500 bloquea el chip si no hay enlace. El envío UDP debe estar **gateado por hardware**: el Arduino sondeará `Ethernet.linkStatus()` y solo intentará enviar el paquete si el cable está físicamente conectado y el switch responde.

### CAP-3: Watchdog de Hardware y Health-Check

El `struct` UDP incluye un contador secuencial `sequence_number`) y un bitmask de salud física `hardware_status`).

* **Criterio de Falla:** Si un sensor I2C sufre un timeout, da NACK, o envía N lecturas idénticas seguidas (congelamiento de bus), su bit de salud baja a `0`.
* **Propagación Inmediata (Fail-Fast):** Si un sensor falla, su variable `float` asociada se seteará forzosamente a `NaN` `0x7FC00000`).
* El servidor Python evalúa esta estructura y dispara el Global Abort.

---

## 4. Contrato de Red (Capa de Aplicación)

### 4.1. Estructura Binaria Estricta (38 Bytes, Little-Endian)

Se exige empaquetado continuo en memoria `__attribute__((packed))`), orden de bytes Little-Endian, y flotantes IEEE 754 de 32 bits.

```cpp

#include <stdint.h>

#include <math.h>

// Contrato Binario Estricto (UDP Payload) - Little Endian

struct __attribute__((packed)) SteeringTelemetry {

    uint32_t sequence_number; // [0-4294967295] Detección de salto/pérdida UDP

    uint32_t timestamp_ms;    // Uptime local. Python asigna Unix Epoch absoluto al recibir.

    // Posición y Orientación (IEEE 754 - 32 bit floats)

    float roll_deg;           // [DIR-01] IMU

    float pitch_deg;          // [DIR-01] IMU

    float heading_deg;        // [DIR-01] IMU

    float inc_pitch_deg;      // [DIR-06] Inclinómetro

    float inc_roll_deg;       // [DIR-06] Inclinómetro

    float encoder_a_deg;      // [DIR-02] Anillo A

    float encoder_b_deg;      // [DIR-03] Anillo B

    // Discretos y Salud (8-bit)

    uint8_t limit_switches;   // Bitmask [Bit0: Tope A, Bit1: Tope B] (1 = Libre/Seguro, 0 = Tope/Cable Roto)

    uint8_t hardware_status;  // Health Bitmask (1=OK, 0=Falla) [Bit0: IMU, Bit1: Inc, Bit2: EncA...]

};
```

### Guardas de Compilación en C++:

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "Requiere Little-Endian");

static_assert(sizeof(float) == 4, "Float debe ser de 32 bits");

static_assert(sizeof(SteeringTelemetry) == 38, "Contrato roto: El tamaño debe ser 38 bytes");

## 5. Advertencias y Alternativas Consideradas (Design Rationale)

> **PROPÓSITO DE ESTA SECCIÓN:** Documentar el *por qué* de las decisiones de diseño arquitectónico. Ningún ingeniero futuro debe "simplificar" estos requerimientos sin entender el modo de falla catastrófica que están previniendo.

1. **Arduino Mega 2560 (5V) vs Teensy 4.1 (3.3V):** Se descartó la potencia de 32-bits del Teensy porque su lógica de 3.3V es altamente susceptible al ruido y requiere *level-shifters* para los sensores industriales. El Mega 2560 opera nativamente a 5V, brindando inmunidad eléctrica superior, lo cual es crítico cerca de motores de 480V.
2. **Muestreo a 50Hz / Transmisión a 10Hz:** Se adquieren datos a alta velocidad (50Hz) para detectar colisiones en los topes mecánicos en 20ms y filtrarlos (debounce) internamente. Sin embargo, la transmisión por red se rebajó a 10Hz para sincronizarse perfectamente con el reloj asíncrono del Servidor Python, evitando la acumulación de datos (buffer bloat) en la red[cite: 19].
3. **I2C Diferencial Obligatorio (PCA9615):** Ante la incertidumbre de la longitud final del cableado dentro del cabezal, el protocolo I2C estándar (Single-Ended) fue prohibido. Un cable I2C largo actúa como antena para el ruido de los variadores (VFD). El transceptor diferencial anula el ruido de modo común, garantizando que la tuneladora no pierda su IMU por EMI.
4. **Fines de Carrera Normalmente Cerrados (NC):** Si se utilizaran contactos NO (Normalmente Abiertos), un cable cortado por la maquinaria sería indistinguible de una pista libre, permitiendo una colisión mecánica a toda potencia. Al usar NC, cualquier cable roto corta la corriente, el Arduino lee un `0`, y detiene la máquina por diseño (Fail-Safe).
5. **Flotantes "NaN" (Not a Number):** En lugar de transmitir el "último valor conocido" o `0.0` cuando un sensor se desconecta, el Arduino inyecta forzosamente un `NaN` matemático en el struct. Esto obliga a la capa Python a colapsar de manera segura y ruidosa (Fail-Fast) si el programador del backend olvidó evaluar el bitmask de salud del sensor, evitando que el algoritmo de navegación opere sobre datos fantasma.
6. **Tamaño del Struct Fijo (38 Bytes):** Se descartó agregar verificaciones redundantes (CRC16 de aplicación) porque la cabecera UDP ya incorpora un *checksum* nativo que descarta tramas corruptas. La detección de cables sueltos se confía al `sequence_number` y al Watchdog del servidor.
