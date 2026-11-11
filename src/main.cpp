#if !defined(SERIAL_TX_BUFFER_SIZE)
#define SERIAL_TX_BUFFER_SIZE 256
#endif

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

// =============================================================================
// Nodo sensorial "dumb" de GNC — Módulo de Dirección, micro-tuneladora B.I.C.H.O.
// Hardware objetivo: Arduino Mega 2560 + (futuro) Ethernet Shield W5500.
//
// REGLA DE ACERO: este firmware solo adquiere y empuja telemetría.
// No evalúa interlocks, no arma salidas de potencia y no comanda el PLC SEW.
// Toda decisión de aborto vive en el servidor central.
//
// Direccionamiento de esta build (directiva de arquitectura, por encima del
// 192.168.1.10 citado en el PRD rev 2.1):
//   origen   192.168.1.11  (este Arduino)
//   destino  192.168.1.2   (Mini PC)  UDP/5000
// El puerto 5000 no está fijado en el PRD; es el puerto de laboratorio.
// Gateway 192.168.1.1 y máscara /24 quedan registrados para el W5500 real.
// =============================================================================

#if defined(ARDUINO) && !defined(__AVR_ATmega2560__)
#error "Este firmware esta fijado al Arduino Mega 2560 (ATmega2560)."
#endif

// -----------------------------------------------------------------------------
// Contrato binario UDP — 38 bytes, Little-Endian, float IEEE 754 binary32
// -----------------------------------------------------------------------------

struct __attribute__((packed)) SteeringTelemetry {
    uint32_t sequence_number;
    uint32_t timestamp_ms;
    float roll_deg;
    float pitch_deg;
    float heading_deg;
    float inc_pitch_deg;
    float inc_roll_deg;
    float encoder_a_deg;
    float encoder_b_deg;
    uint8_t limit_switches;
    uint8_t hardware_status;
};

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "Requiere Little-Endian");
static_assert(sizeof(float) == 4, "Float debe ser de 32 bits");
static_assert(__FLT_RADIX__ == 2, "Float debe ser IEEE 754 binario");
static_assert(__FLT_MANT_DIG__ == 24, "Float debe ser IEEE 754 binary32");
static_assert(__FLT_MAX_EXP__ == 128, "Float debe ser IEEE 754 binary32");
static_assert(sizeof(SteeringTelemetry) == 38, "Contrato roto: El tamaño debe ser 38 bytes");

static_assert(offsetof(SteeringTelemetry, sequence_number) == 0, "Offset sequence_number");
static_assert(offsetof(SteeringTelemetry, timestamp_ms) == 4, "Offset timestamp_ms");
static_assert(offsetof(SteeringTelemetry, roll_deg) == 8, "Offset roll_deg");
static_assert(offsetof(SteeringTelemetry, pitch_deg) == 12, "Offset pitch_deg");
static_assert(offsetof(SteeringTelemetry, heading_deg) == 16, "Offset heading_deg");
static_assert(offsetof(SteeringTelemetry, inc_pitch_deg) == 20, "Offset inc_pitch_deg");
static_assert(offsetof(SteeringTelemetry, inc_roll_deg) == 24, "Offset inc_roll_deg");
static_assert(offsetof(SteeringTelemetry, encoder_a_deg) == 28, "Offset encoder_a_deg");
static_assert(offsetof(SteeringTelemetry, encoder_b_deg) == 32, "Offset encoder_b_deg");
static_assert(offsetof(SteeringTelemetry, limit_switches) == 36, "Offset limit_switches");
static_assert(offsetof(SteeringTelemetry, hardware_status) == 37, "Offset hardware_status");

// hardware_status: 1 = sano, 0 = falla. El nibble alto queda en 1 (reservado,
// sin falla declarada) para que el servidor enmascare solo los bits definidos.
constexpr uint8_t kHealthImu = 0x01u;        // Bit0 DIR-01 IMU (roll, pitch, heading)
constexpr uint8_t kHealthInc = 0x02u;        // Bit1 DIR-06 Inclinómetro (pitch, roll)
constexpr uint8_t kHealthEncA = 0x04u;       // Bit2 DIR-02 Encoder anillo A
constexpr uint8_t kHealthEncB = 0x08u;       // Bit3 DIR-03 Encoder anillo B
constexpr uint8_t kHealthReserved = 0xF0u;   // Bits 4..7 reservados = sanos

// limit_switches: 1 = libre/seguro, 0 = tope o cable abierto (NC fail-safe).
constexpr uint8_t kLimitPos10 = 0x01u;       // Bit0 Tope A +10°
constexpr uint8_t kLimitNeg10 = 0x02u;       // Bit1 Tope B -10°

constexpr uint32_t kAcquirePeriodMs = 20u;   // 50 Hz
constexpr uint32_t kTransmitPeriodMs = 100u; // 10 Hz
constexpr uint8_t kDebounceIdenticalReadings = 4u; // 4 muestras a 50 Hz: estabilidad >= 50 ms

constexpr uint16_t kLocalUdpPort = 5000u;
constexpr uint16_t kRemoteUdpPort = 5000u;

constexpr float kTwoPi = 6.28318530718f;

struct NetworkConfig {
    uint8_t mac[6];
    uint8_t localIp[4];
    uint8_t gateway[4];
    uint8_t subnet[4];
    uint8_t remoteIp[4];
    uint16_t localPort;
    uint16_t remotePort;
};

// MAC localmente administrada (bit U/L = 1, bit multicast = 0): "TLDIR".
const NetworkConfig kNetworkConfig = {
    {0x02u, 0x54u, 0x4Cu, 0x44u, 0x49u, 0x52u},
    {192u, 168u, 1u, 11u},
    {192u, 168u, 1u, 1u},
    {255u, 255u, 255u, 0u},
    {192u, 168u, 1u, 2u},
    kLocalUdpPort,
    kRemoteUdpPort
};

// -----------------------------------------------------------------------------
// Interfaces puras. Los objetos concretos viven en almacenamiento estático.
// Prohibido new, malloc y String. La inyección es por referencia.
// -----------------------------------------------------------------------------

class ISensor {
public:
    virtual float read() = 0;
    virtual bool isHealthy() const = 0;
    virtual ~ISensor() = default;

protected:
    ISensor() = default;
    ISensor(const ISensor&) = delete;
    ISensor& operator=(const ISensor&) = delete;
};

class IDiscreteInput {
public:
    // 1 = circuito NC cerrado (libre). 0 = tope, cable cortado o entrada abierta.
    virtual uint8_t readRaw() = 0;
    virtual ~IDiscreteInput() = default;

protected:
    IDiscreteInput() = default;
    IDiscreteInput(const IDiscreteInput&) = delete;
    IDiscreteInput& operator=(const IDiscreteInput&) = delete;
};

class INetwork {
public:
    virtual bool transmit(uint8_t* payload, size_t size) = 0;
    virtual bool isLinkUp() const = 0;
    virtual ~INetwork() = default;

protected:
    INetwork() = default;
    INetwork(const INetwork&) = delete;
    INetwork& operator=(const INetwork&) = delete;
};

// -----------------------------------------------------------------------------
// Utilidades de contrato (sin libc de copia: no se incluye string.h)
// -----------------------------------------------------------------------------

static float sampleSineDeg(float amplitudeDeg, float freqHz, float phaseRad, float offsetDeg) {
    const float tSec = static_cast<float>(millis()) * 0.001f;
    const float theta = (kTwoPi * freqHz * tSec) + phaseRad;
    const float wave = static_cast<float>(sin(static_cast<double>(theta)));
    return offsetDeg + (amplitudeDeg * wave);
}

static uint32_t readU32LE(const uint8_t* bytes) {
    return static_cast<uint32_t>(bytes[0])
        | (static_cast<uint32_t>(bytes[1]) << 8)
        | (static_cast<uint32_t>(bytes[2]) << 16)
        | (static_cast<uint32_t>(bytes[3]) << 24);
}

static float readF32LE(const uint8_t* bytes) {
    union {
        uint32_t u;
        float f;
    } value;
    value.u = readU32LE(bytes);
    return value.f;
}

static void applyFailFast(SteeringTelemetry& packet) {
    const uint8_t health = packet.hardware_status;
    if ((health & kHealthImu) == 0u) {
        packet.roll_deg = NAN;
        packet.pitch_deg = NAN;
        packet.heading_deg = NAN;
    }
    if ((health & kHealthInc) == 0u) {
        packet.inc_pitch_deg = NAN;
        packet.inc_roll_deg = NAN;
    }
    if ((health & kHealthEncA) == 0u) {
        packet.encoder_a_deg = NAN;
    }
    if ((health & kHealthEncB) == 0u) {
        packet.encoder_b_deg = NAN;
    }
}

static void loadFailSafeTelemetry(SteeringTelemetry& packet) {
    packet.sequence_number = 0u;
    packet.timestamp_ms = 0u;
    packet.roll_deg = NAN;
    packet.pitch_deg = NAN;
    packet.heading_deg = NAN;
    packet.inc_pitch_deg = NAN;
    packet.inc_roll_deg = NAN;
    packet.encoder_a_deg = NAN;
    packet.encoder_b_deg = NAN;
    packet.limit_switches = 0u;
    packet.hardware_status = kHealthReserved;
}

// -----------------------------------------------------------------------------
// Debounce NC + latch. El 0 validado permanece hasta que la trama UDP sale,
// para que un impacto entre dos envíos de 10 Hz no se pierda.
// -----------------------------------------------------------------------------

class DebouncedNcChannel {
public:
    DebouncedNcChannel()
        : stable_(0u),
          candidate_(0u),
          identicalCount_(0u),
          latchHit_(0u) {}

    void sample(uint8_t rawLevel) {
        const uint8_t level = (rawLevel == 0u) ? 0u : 1u;
        if (identicalCount_ == 0u) {
            candidate_ = level;
            identicalCount_ = 1u;
        } else if (level == candidate_) {
            if (identicalCount_ < kDebounceIdenticalReadings) {
                identicalCount_++;
            }
        } else {
            candidate_ = level;
            identicalCount_ = 1u;
        }

        if ((identicalCount_ >= kDebounceIdenticalReadings) && (stable_ != candidate_)) {
            stable_ = candidate_;
            if (stable_ == 0u) {
                latchHit_ = 1u;
            }
        }
    }

    uint8_t reportedLevel() const {
        if ((stable_ == 0u) || (latchHit_ != 0u)) {
            return 0u;
        }
        return 1u;
    }

    void releaseLatchIfSafe() {
        if (stable_ == 1u) {
            latchHit_ = 0u;
        }
    }

private:
    uint8_t stable_;
    uint8_t candidate_;
    uint8_t identicalCount_;
    uint8_t latchHit_;
};

// -----------------------------------------------------------------------------
// Dummies. Sustituyen I2C (PCA9615 / IMU / inclinómetro) y los encoders
// absolutos hasta integrar Wire. No bloquean y no tocan el bus.
// -----------------------------------------------------------------------------

class SineAxis final : public ISensor {
public:
    SineAxis(float amplitudeDeg, float freqHz, float phaseRad, float offsetDeg)
        : amplitudeDeg_(amplitudeDeg),
          freqHz_(freqHz),
          phaseRad_(phaseRad),
          offsetDeg_(offsetDeg),
          healthy_(true) {}

    float read() override {
        return sampleSineDeg(amplitudeDeg_, freqHz_, phaseRad_, offsetDeg_);
    }

    bool isHealthy() const override {
        return healthy_;
    }

private:
    float amplitudeDeg_;
    float freqHz_;
    float phaseRad_;
    float offsetDeg_;
    bool healthy_;
};

class DummyIMU {
public:
    DummyIMU(float rollAmpDeg, float rollHz, float rollPhaseRad,
             float pitchAmpDeg, float pitchHz, float pitchPhaseRad,
             float headingAmpDeg, float headingHz, float headingPhaseRad)
        : roll_(rollAmpDeg, rollHz, rollPhaseRad, 0.0f),
          pitch_(pitchAmpDeg, pitchHz, pitchPhaseRad, 0.0f),
          heading_(headingAmpDeg, headingHz, headingPhaseRad, 0.0f) {}

    ISensor& roll() { return roll_; }
    ISensor& pitch() { return pitch_; }
    ISensor& heading() { return heading_; }

private:
    SineAxis roll_;
    SineAxis pitch_;
    SineAxis heading_;
};

class DummyInclinometer {
public:
    DummyInclinometer(float pitchAmpDeg, float pitchHz, float pitchPhaseRad,
                      float rollAmpDeg, float rollHz, float rollPhaseRad)
        : pitch_(pitchAmpDeg, pitchHz, pitchPhaseRad, 0.15f),
          roll_(rollAmpDeg, rollHz, rollPhaseRad, -0.10f) {}

    ISensor& pitch() { return pitch_; }
    ISensor& roll() { return roll_; }

private:
    SineAxis pitch_;
    SineAxis roll_;
};

class DummyEncoder final : public ISensor {
public:
    DummyEncoder(float amplitudeDeg, float freqHz, float phaseRad, float offsetDeg)
        : amplitudeDeg_(amplitudeDeg),
          freqHz_(freqHz),
          phaseRad_(phaseRad),
          offsetDeg_(offsetDeg),
          healthy_(true) {}

    float read() override {
        return sampleSineDeg(amplitudeDeg_, freqHz_, phaseRad_, offsetDeg_);
    }

    bool isHealthy() const override {
        return healthy_;
    }

private:
    float amplitudeDeg_;
    float freqHz_;
    float phaseRad_;
    float offsetDeg_;
    bool healthy_;
};

class DummyLimitSwitch final : public IDiscreteInput {
public:
    DummyLimitSwitch(uint32_t periodMs, uint32_t assertAtMs, uint32_t assertForMs)
        : periodMs_(periodMs),
          assertAtMs_(assertAtMs),
          assertForMs_(assertForMs) {}

    uint8_t readRaw() override {
        if ((periodMs_ == 0u) || (assertForMs_ == 0u)) {
            return 1u;
        }
        const uint32_t phase = static_cast<uint32_t>(millis()) % periodMs_;
        const uint32_t assertEnd = assertAtMs_ + assertForMs_;
        if (assertEnd <= periodMs_) {
            if ((phase >= assertAtMs_) && (phase < assertEnd)) {
                return 0u;
            }
            return 1u;
        }
        const uint32_t wrappedEnd = assertEnd % periodMs_;
        if ((phase >= assertAtMs_) || (phase < wrappedEnd)) {
            return 0u;
        }
        return 1u;
    }

private:
    uint32_t periodMs_;
    uint32_t assertAtMs_;
    uint32_t assertForMs_;
};

class DummyUDPNetwork final : public INetwork {
public:
    explicit DummyUDPNetwork(const NetworkConfig& config)
        : localPort_(config.localPort),
          remotePort_(config.remotePort),
          linkUp_(false),
          lastSize_(0u) {
        copyBytes(mac_, config.mac, 6u);
        copyBytes(localIp_, config.localIp, 4u);
        copyBytes(gateway_, config.gateway, 4u);
        copyBytes(subnet_, config.subnet, 4u);
        copyBytes(remoteIp_, config.remoteIp, 4u);
        clearPayload();
    }

    void begin() {
        Serial.begin(115200);
        linkUp_ = true;
        Serial.print(F("GNC DIR solo-lectura Mega2560 payload="));
        Serial.println(static_cast<unsigned>(sizeof(SteeringTelemetry)));
        Serial.print(F("IP local "));
        printIp(localIp_);
        Serial.print(F(" gw "));
        printIp(gateway_);
        Serial.print(F(" mask "));
        printIp(subnet_);
        Serial.print(F(" mac "));
        printMac(mac_);
        Serial.print(F("\nUDP -> "));
        printIp(remoteIp_);
        Serial.print(':');
        Serial.print(remotePort_);
        Serial.print(F(" src-port "));
        Serial.println(localPort_);
        union {
            float f;
            uint32_t u;
        } nanProbe;
        nanProbe.f = NAN;
        Serial.print(F("NAN bits 0x"));
        printHex32(nanProbe.u);
        Serial.print(F(" enlace="));
        Serial.println(linkUp_ ? F("UP") : F("DOWN"));
        Serial.println(F("cols: seq ms lim hs roll pitch head incP incR encA encB"));
    }

    bool transmit(uint8_t* payload, size_t size) override {
        if ((payload == nullptr) || (size != sizeof(SteeringTelemetry)) || !linkUp_) {
            return false;
        }
        for (size_t i = 0u; i < size; ++i) {
            lastPayload_[i] = payload[i];
        }
        lastSize_ = size;
        mirrorToSerial(lastPayload_, lastSize_);
        return true;
    }

    bool isLinkUp() const override {
        return linkUp_;
    }

private:
    static void copyBytes(uint8_t* dest, const uint8_t* src, size_t count) {
        for (size_t i = 0u; i < count; ++i) {
            dest[i] = src[i];
        }
    }

    void clearPayload() {
        for (size_t i = 0u; i < sizeof(lastPayload_); ++i) {
            lastPayload_[i] = 0u;
        }
    }

    void printIp(const uint8_t ip[4]) const {
        Serial.print(ip[0]);
        Serial.print('.');
        Serial.print(ip[1]);
        Serial.print('.');
        Serial.print(ip[2]);
        Serial.print('.');
        Serial.print(ip[3]);
    }

    void printMac(const uint8_t mac[6]) const {
        for (uint8_t i = 0u; i < 6u; ++i) {
            if (i != 0u) {
                Serial.print(':');
            }
            printHex8(mac[i]);
        }
    }

    void printHex8(uint8_t value) const {
        static const char kDigits[] = "0123456789ABCDEF";
        Serial.print(kDigits[(value >> 4) & 0x0Fu]);
        Serial.print(kDigits[value & 0x0Fu]);
    }

    void printHex32(uint32_t value) const {
        printHex8(static_cast<uint8_t>((value >> 24) & 0xFFu));
        printHex8(static_cast<uint8_t>((value >> 16) & 0xFFu));
        printHex8(static_cast<uint8_t>((value >> 8) & 0xFFu));
        printHex8(static_cast<uint8_t>(value & 0xFFu));
    }

    void mirrorFull(const uint8_t* payload) const {
        Serial.print(readU32LE(payload + 0));
        Serial.print(' ');
        Serial.print(readU32LE(payload + 4));
        Serial.print(' ');
        printHex8(payload[36]);
        Serial.print(' ');
        printHex8(payload[37]);
        Serial.print(' ');
        Serial.print(readF32LE(payload + 8), 2);
        Serial.print(' ');
        Serial.print(readF32LE(payload + 12), 2);
        Serial.print(' ');
        Serial.print(readF32LE(payload + 16), 2);
        Serial.print(' ');
        Serial.print(readF32LE(payload + 20), 2);
        Serial.print(' ');
        Serial.print(readF32LE(payload + 24), 2);
        Serial.print(' ');
        Serial.print(readF32LE(payload + 28), 2);
        Serial.print(' ');
        Serial.println(readF32LE(payload + 32), 2);
    }

    void mirrorShort(const uint8_t* payload) const {
        Serial.print(readU32LE(payload + 0));
        Serial.print(' ');
        Serial.print(readU32LE(payload + 4));
        Serial.print(' ');
        printHex8(payload[36]);
        Serial.print(' ');
        printHex8(payload[37]);
        Serial.print(' ');
        Serial.print(readF32LE(payload + 28), 2);
        Serial.print(' ');
        Serial.println(readF32LE(payload + 32), 2);
    }

    void mirrorToSerial(const uint8_t* payload, size_t size) const {
        if (size != sizeof(SteeringTelemetry)) {
            return;
        }
        // No se espera al UART. Con buffer de 256 entra la línea completa;
        // con el buffer por defecto (64) entra la línea corta. Si no hay
        // espacio, el datagrama igual queda en lastPayload_.
        const int room = Serial.availableForWrite();
        if (room >= 200) {
            mirrorFull(payload);
            return;
        }
        if (room >= 56) {
            mirrorShort(payload);
        }
    }

    uint8_t mac_[6];
    uint8_t localIp_[4];
    uint8_t gateway_[4];
    uint8_t subnet_[4];
    uint8_t remoteIp_[4];
    uint16_t localPort_;
    uint16_t remotePort_;
    bool linkUp_;
    uint8_t lastPayload_[sizeof(SteeringTelemetry)];
    size_t lastSize_;
};

// -----------------------------------------------------------------------------
// Aplicación. Dos relojes millis() independientes. El de 50 Hz solo muestrea;
// el de 10 Hz solo empaqueta el último valor conocido.
// -----------------------------------------------------------------------------

class SteeringNode {
public:
    SteeringNode(ISensor& imuRoll,
                 ISensor& imuPitch,
                 ISensor& imuHeading,
                 ISensor& incPitch,
                 ISensor& incRoll,
                 ISensor& encoderA,
                 ISensor& encoderB,
                 IDiscreteInput& limitPos10,
                 IDiscreteInput& limitNeg10,
                 INetwork& network)
        : imuRoll_(imuRoll),
          imuPitch_(imuPitch),
          imuHeading_(imuHeading),
          incPitch_(incPitch),
          incRoll_(incRoll),
          encoderA_(encoderA),
          encoderB_(encoderB),
          limitPos10_(limitPos10),
          limitNeg10_(limitNeg10),
          network_(network),
          sequence_(0u),
          lastAcquireMs_(0u),
          lastTransmitMs_(0u),
          clockPrimed_(false) {
        loadFailSafeTelemetry(latest_);
    }

    SteeringNode(const SteeringNode&) = delete;
    SteeringNode& operator=(const SteeringNode&) = delete;

    void begin() {
        const uint32_t now = static_cast<uint32_t>(millis());
        lastAcquireMs_ = now;
        lastTransmitMs_ = now;
        clockPrimed_ = true;
    }

    void tick() {
        if (!clockPrimed_) {
            begin();
        }
        const uint32_t now = static_cast<uint32_t>(millis());
        const uint32_t sinceAcquire = static_cast<uint32_t>(now - lastAcquireMs_);
        if (sinceAcquire >= kAcquirePeriodMs) {
            if (sinceAcquire >= (kAcquirePeriodMs * 2u)) {
                lastAcquireMs_ = now;
            } else {
                lastAcquireMs_ += kAcquirePeriodMs;
            }
            acquire();
        }
        const uint32_t sinceTransmit = static_cast<uint32_t>(now - lastTransmitMs_);
        if (sinceTransmit >= kTransmitPeriodMs) {
            if (sinceTransmit >= (kTransmitPeriodMs * 2u)) {
                lastTransmitMs_ = now;
            } else {
                lastTransmitMs_ += kTransmitPeriodMs;
            }
            transmit(now);
        }
    }

private:
    bool acceptChannel(ISensor& sensor, float& lastKnown) {
        if (!sensor.isHealthy()) {
            return false;
        }
        const float sample = sensor.read();
        if (isnan(sample) || isinf(sample)) {
            return false;
        }
        lastKnown = sample;
        return true;
    }

    void acquire() {
        const bool rollOk = acceptChannel(imuRoll_, latest_.roll_deg);
        const bool pitchOk = acceptChannel(imuPitch_, latest_.pitch_deg);
        const bool headingOk = acceptChannel(imuHeading_, latest_.heading_deg);
        const bool incPitchOk = acceptChannel(incPitch_, latest_.inc_pitch_deg);
        const bool incRollOk = acceptChannel(incRoll_, latest_.inc_roll_deg);
        const bool encAOk = acceptChannel(encoderA_, latest_.encoder_a_deg);
        const bool encBOk = acceptChannel(encoderB_, latest_.encoder_b_deg);

        uint8_t health = kHealthReserved;
        if (rollOk && pitchOk && headingOk) {
            health = static_cast<uint8_t>(health | kHealthImu);
        }
        if (incPitchOk && incRollOk) {
            health = static_cast<uint8_t>(health | kHealthInc);
        }
        if (encAOk) {
            health = static_cast<uint8_t>(health | kHealthEncA);
        }
        if (encBOk) {
            health = static_cast<uint8_t>(health | kHealthEncB);
        }
        latest_.hardware_status = health;

        limitPos10State_.sample(limitPos10_.readRaw());
        limitNeg10State_.sample(limitNeg10_.readRaw());

        uint8_t limits = 0u;
        if (limitPos10State_.reportedLevel() == 1u) {
            limits = static_cast<uint8_t>(limits | kLimitPos10);
        }
        if (limitNeg10State_.reportedLevel() == 1u) {
            limits = static_cast<uint8_t>(limits | kLimitNeg10);
        }
        latest_.limit_switches = limits;
    }

    void transmit(uint32_t nowMs) {
        SteeringTelemetry packet = latest_;
        packet.sequence_number = sequence_;
        sequence_++;
        packet.timestamp_ms = nowMs;
        applyFailFast(packet);

        if (!network_.isLinkUp()) {
            return;
        }

        uint8_t* raw = reinterpret_cast<uint8_t*>(&packet);
        const bool sent = network_.transmit(raw, sizeof(packet));
        if (sent) {
            limitPos10State_.releaseLatchIfSafe();
            limitNeg10State_.releaseLatchIfSafe();
        }
    }

    ISensor& imuRoll_;
    ISensor& imuPitch_;
    ISensor& imuHeading_;
    ISensor& incPitch_;
    ISensor& incRoll_;
    ISensor& encoderA_;
    ISensor& encoderB_;
    IDiscreteInput& limitPos10_;
    IDiscreteInput& limitNeg10_;
    INetwork& network_;

    DebouncedNcChannel limitPos10State_;
    DebouncedNcChannel limitNeg10State_;

    SteeringTelemetry latest_;
    uint32_t sequence_;
    uint32_t lastAcquireMs_;
    uint32_t lastTransmitMs_;
    bool clockPrimed_;
};

// -----------------------------------------------------------------------------
// Instancias estáticas. Los anillos A/B siguen la misma cuña (±10°) con holgura.
// Los topes dummy se abren (0 lógico) durante >50 ms para ejercer el debounce.
// -----------------------------------------------------------------------------

static DummyIMU gImu(
    2.0f, 0.05f, 0.00f,
    1.5f, 0.07f, 0.40f,
    8.0f, 0.02f, 1.10f);

static DummyInclinometer gInclinometer(
    1.2f, 0.07f, 0.55f,
    1.8f, 0.05f, 0.25f);

static DummyEncoder gEncoderA(10.0f, 0.10f, 0.00f, 0.0f);
static DummyEncoder gEncoderB(9.6f, 0.10f, 0.05f, 0.2f);

static DummyLimitSwitch gLimitPos10(5000u, 2500u, 100u);
static DummyLimitSwitch gLimitNeg10(8000u, 5200u, 120u);

static DummyUDPNetwork gNetwork(kNetworkConfig);

static SteeringNode gNode(
    gImu.roll(),
    gImu.pitch(),
    gImu.heading(),
    gInclinometer.pitch(),
    gInclinometer.roll(),
    gEncoderA,
    gEncoderB,
    gLimitPos10,
    gLimitNeg10,
    gNetwork);

void setup() {
    gNetwork.begin();
    gNode.begin();
}

void loop() {
    gNode.tick();
}
