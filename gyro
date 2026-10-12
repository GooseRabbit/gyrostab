// ICM-20948 axis / angle check  --  YOUR sketch with MINIMAL fixes. Structure, names and your d0..d3 formulas are kept.
// Every change is tagged [FIXn]; the numbers match the findings list in the answer:
//   [FIX1] reference: no longer taken from the first DMP packet; averaged on command 'z', rejected if the board moves
//   [FIX2] angle: no hard-wired Y; twist+swing about AXIS_B (zeros = not set -> no angle); convention switch
//   [FIX3] quaternion: validated (norm), rebuilt in double, renormalised; bad frames are dropped and counted
//   [FIX4] FIFO: Quat6 / Gyro handled independently; FIFO drained; statuses counted; no blocking delay(100)
//   [FIX5] setup: every DMP step checked ('&='), resetDMP() added (as in the SparkFun example)
//   [FIX6] diagnostics: full gyro vector, GyroInt (cross-check), dQ (DMP settling), error counters
// NOT compiled for ESP32 and NOT run on hardware: it was only built against a mock of the library on a PC.

#include <Arduino.h>
#include <SPI.h>
#include <ICM_20948.h>
#include <ESP32Servo.h>


// ============================================================
// PINS
// ============================================================

#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define SPI_CS     5

// ============================================================
// GYRO
// ============================================================

const float GYRO_SCALE = 2000.0f / 32768.0f;
const float GYRO_ALPHA = 0.20f;

// ============================================================
// TIMING
// ============================================================

const uint32_t CONTROL_PERIOD_US = 5000;
const uint32_t DMP_TIMEOUT_MS = 200;

// ============================================================
// IMU
// ============================================================

ICM_20948_SPI myICM;

// ============================================================
// [FIX2] shaft axis in CHIP axes + quaternion convention
// ============================================================
Servo myServo;
String servoCommand;

// Unit vector along the shaft, in chip axes X,Y,Z (DS-000189 fig. 12). All zeros = NOT SET:
// the angle is then not computed (no axis is assumed). Fill it from geometry / the gyro experiment.
const float AXIS_B[3] = { 0.0f, 1.0f, 0.0f };

// 1: DMP quaternion = body->world (assumed, not proven). 0: world->body.
// Change ONLY if the gyro cross-check (Twist vs GyroInt) says so, with a non-identity reference.
#define QUAT_IS_BODY_TO_WORLD 1

const int   REF_SAMPLES    = 100;    // packets averaged for the reference and the gyro bias (about 2 s)
const float REF_MAX_DEV_DEG = 0.5f;  // reference is rejected if q moved more than this while averaging

// ============================================================
// QUATERNION
// ============================================================

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float q0Start = 1.0f;
float q1Start = 0.0f;
float q2Start = 0.0f;
float q3Start = 0.0f;

bool orientationInitialized = false;

float roll = 0.0f;          // now: TWIST about AXIS_B (deg), not "roll about Y"
float swingDeg = 0.0f;      // how far the shaft axis itself has moved (deg): validity indicator

float axisN[3] = { 0.0f, 0.0f, 0.0f };
bool  axisSet = false;

// [FIX1] reference capture (manual: send 'z' when the board is still and the DMP has settled)
bool  refRequested = false;
int   refQn = 0, refGn = 0;
float refQ0[4]  = { 1, 0, 0, 0 };    // first sample of the window
float refQacc[4] = { 0, 0, 0, 0 };   // sign-aligned sum
float refMaxDev = 0.0f;
float refGacc[3] = { 0, 0, 0 };
float gBias[3]   = { 0, 0, 0 };
float gW[3]      = { 0, 0, 0 };      // gyro minus bias, dps, chip X,Y,Z

// [FIX6] gyro cross-check
float    rateAlongAxis = 0.0f, gSumRate = 0.0f, gyroIntDeg = 0.0f;
uint32_t gN = 0, tG0 = 0, tGLast = 0;

uint32_t badNorm = 0, statusErrors = 0;

// ============================================================
// GYRO STATE
// ============================================================

float gyroRate = 0.0f;
float gyroRateFiltered = 0.0f;

bool gyroFilterInitialized = false;

// ============================================================
// DMP STATE
// ============================================================

uint32_t lastDMPTime = 0;
uint32_t dmpCounter = 0;
uint32_t lastControlTime = 0;

// ============================================================
// PROCESS DMP PACKET
// ============================================================

// [FIX1/6] angle (rad) between two unit quaternions given as float[4]; sign-insensitive, well conditioned near 0 (no acos)
static float qAngleF(const float a[4], const float b[4])
{
    const float w = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];                 // conj(a) * b
    const float x = a[0]*b[1] - a[1]*b[0] - a[2]*b[3] + a[3]*b[2];
    const float y = a[0]*b[2] + a[1]*b[3] - a[2]*b[0] - a[3]*b[1];
    const float z = a[0]*b[3] - a[1]*b[2] + a[2]*b[1] - a[3]*b[0];
    return 2.0f * atan2f(sqrtf(x*x + y*y + z*z), fabsf(w));
}

static void finishReference()
{
    float n = sqrtf(refQacc[0]*refQacc[0] + refQacc[1]*refQacc[1] + refQacc[2]*refQacc[2] + refQacc[3]*refQacc[3]);
    q0Start = refQacc[0]/n; q1Start = refQacc[1]/n; q2Start = refQacc[2]/n; q3Start = refQacc[3]/n;
    for (int i = 0; i < 3; i++) gBias[i] = refGacc[i] / refGn;
    gN = 0; gSumRate = 0.0f; gyroIntDeg = 0.0f;
    orientationInitialized = true;
    refRequested = false;
    Serial.print("REF captured. q=");
    Serial.print(q0Start, 4); Serial.print(","); Serial.print(q1Start, 4); Serial.print(",");
    Serial.print(q2Start, 4); Serial.print(","); Serial.print(q3Start, 4);
    Serial.print(" gyroBias(dps)="); Serial.print(gBias[0], 2); Serial.print(",");
    Serial.print(gBias[1], 2); Serial.print(","); Serial.println(gBias[2], 2);
}

// [FIX2]
static void computeAngle()
{
    // Relative rotation expressed in BODY axes.
#if QUAT_IS_BODY_TO_WORLD
    // qDelta = conjugate(qStart) * qCurrent      (your original formulas, unchanged)
    float d0 = q0Start*q0 + q1Start*q1 + q2Start*q2 + q3Start*q3;
    float d1 = q0Start*q1 - q1Start*q0 - q2Start*q3 + q3Start*q2;
    float d2 = q0Start*q2 + q1Start*q3 - q2Start*q0 - q3Start*q1;
    float d3 = q0Start*q3 - q1Start*q2 + q2Start*q1 - q3Start*q0;
#else
    // qDelta = qStart * conjugate(qCurrent)
    float d0 =  q0Start*q0 + q1Start*q1 + q2Start*q2 + q3Start*q3;
    float d1 = -q0Start*q1 + q1Start*q0 - q2Start*q3 + q3Start*q2;
    float d2 = -q0Start*q2 + q1Start*q3 + q2Start*q0 - q3Start*q1;
    float d3 = -q0Start*q3 - q1Start*q2 + q2Start*q1 + q3Start*q0;
#endif
    if (d0 < 0.0f) { d0 = -d0; d1 = -d1; d2 = -d2; d3 = -d3; }   // shortest branch (unchanged)

    if (!axisSet) return;                                         // no axis assumed

    // swing-twist about AXIS_B:  twist = 2*atan2(n.v, w);  swing = how far the axis itself moved
    const float p  = d1*axisN[0] + d2*axisN[1] + d3*axisN[2];
    const float ux = d1 - p*axisN[0], uy = d2 - p*axisN[1], uz = d3 - p*axisN[2];
    const float perp = sqrtf(ux*ux + uy*uy + uz*uz);
    const float m    = sqrtf(d0*d0 + p*p);
    if (m < 1e-4f) return;                                        // swing ~180 deg: twist undefined
    roll     = 2.0f * atan2f(p, d0)    * 180.0f / PI;
    swingDeg = 2.0f * atan2f(perp, m)  * 180.0f / PI;
}

bool processDMPPacket(icm_20948_DMP_data_t &data)
{
    // [FIX4] Quat6 and Gyro are handled independently: a frame lacking one of them is no longer thrown away.
    const bool gotQuat = (data.header & DMP_header_bitmap_Quat6) != 0;
    const bool gotGyro = (data.header & DMP_header_bitmap_Gyro)  != 0;

    if (!gotQuat && !gotGyro)
        return false;

    const uint32_t nowUs = micros();

    if (gotGyro)
    {
        const float gx = (float)data.Raw_Gyro.Data.X * GYRO_SCALE;
        const float gy = (float)data.Raw_Gyro.Data.Y * GYRO_SCALE;
        const float gz = (float)data.Raw_Gyro.Data.Z * GYRO_SCALE;

        gyroRate = gy;                       // as before: raw gyro Y

        if (!gyroFilterInitialized) { gyroRateFiltered = gyroRate; gyroFilterInitialized = true; }
        else gyroRateFiltered = gyroRateFiltered * (1.0f - GYRO_ALPHA) + gyroRate * GYRO_ALPHA;

        if (refRequested && refGn < REF_SAMPLES) { refGacc[0] += gx; refGacc[1] += gy; refGacc[2] += gz; refGn++; }

        gW[0] = gx - gBias[0]; gW[1] = gy - gBias[1]; gW[2] = gz - gBias[2];

        if (orientationInitialized && axisSet)
        {
            rateAlongAxis = gW[0]*axisN[0] + gW[1]*axisN[1] + gW[2]*axisN[2];
            if (gN == 0) tG0 = nowUs;
            tGLast = nowUs; gN++; gSumRate += rateAlongAxis;
            const float dt = (gN > 20 && tGLast != tG0) ? (float)(uint32_t)(tGLast - tG0) * 1e-6f / (float)(gN - 1)
                                                         : (1.0f / 56.25f);   // measured mean packet period
            gyroIntDeg = gSumRate * dt;
        }
    }

    if (gotQuat)
    {
        // [FIX3] Quat6 components are scaled by 2^30; the 4th component is rebuilt from the norm (double: no cancellation).
        const double sc = 1.0 / 1073741824.0;
        const double x = (double)data.Quat6.Data.Q1 * sc;
        const double y = (double)data.Quat6.Data.Q2 * sc;
        const double z = (double)data.Quat6.Data.Q3 * sc;
        double r = 1.0 - (x*x + y*y + z*z);

        if (r < -0.02)
        {
            badNorm++;                       // norm violated: corrupted frame, do not use it
        }
        else
        {
            if (r < 0.0) r = 0.0;
            const double w = sqrt(r);
            const double n = sqrt(w*w + x*x + y*y + z*z);     // renormalise after the clamp
            q0 = (float)(w / n); q1 = (float)(x / n); q2 = (float)(y / n); q3 = (float)(z / n);

            // [FIX1] reference: average REF_SAMPLES sign-aligned packets, reject if the board moved meanwhile
            if (refRequested && refQn < REF_SAMPLES)
            {
                const float qc[4] = { q0, q1, q2, q3 };
                if (refQn == 0) { for (int i = 0; i < 4; i++) { refQ0[i] = qc[i]; refQacc[i] = 0.0f; } refMaxDev = 0.0f; }
                const float dev = qAngleF(refQ0, qc) * 180.0f / PI;
                if (dev > refMaxDev) refMaxDev = dev;
                const float sgn = (refQ0[0]*qc[0] + refQ0[1]*qc[1] + refQ0[2]*qc[2] + refQ0[3]*qc[3]) < 0.0f ? -1.0f : 1.0f;
                for (int i = 0; i < 4; i++) refQacc[i] += sgn * qc[i];
                refQn++;
            }
            if (refRequested && refQn >= REF_SAMPLES && refGn >= REF_SAMPLES)
            {
                if (refMaxDev > REF_MAX_DEV_DEG)
                {
                    Serial.print("REF REJECTED: q moved by "); Serial.print(refMaxDev, 2);
                    Serial.println(" deg during capture (not still / DMP still settling). Send 'z' again.");
                    refRequested = false;
                }
                else finishReference();
            }

            if (orientationInitialized) computeAngle();
        }
    }

    lastDMPTime = millis();
    dmpCounter++;

    return true;
}

// ============================================================
// READ FIFO
// ============================================================

// [FIX4]
void readDMP()
{
    icm_20948_DMP_data_t data;

    for (int i = 0; i < 64; i++)
    {
        memset(&data, 0, sizeof(data));                    // no leftovers from a previous frame
        myICM.readDMPdataFromFIFO(&data);
        const ICM_20948_Status_e st = myICM.status;

        if (st == ICM_20948_Stat_FIFONoDataAvail)
            break;                                          // FIFO is empty

        if (st != ICM_20948_Stat_Ok && st != ICM_20948_Stat_FIFOMoreDataAvail)
        {
            statusErrors++;                                 // incomplete frame / SPI error: count it, do not use data
            break;
        }

        processDMPPacket(data);

        if (st == ICM_20948_Stat_Ok)
            break;                                          // valid frame and nothing more in the FIFO
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("ICM-20948 AXIS / ANGLE CHECK");
    Serial.println("SERVO CONTROL DISABLED");
    Serial.println("========================================");

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

    Serial.println("Starting ICM-20948...");

    myICM.begin(SPI_CS, SPI);
    myServo.setPeriodHertz(50);
    myServo.attach(25, 1100, 1900);
    myServo.writeMicroseconds(1500);

    Serial.print("IMU status: ");
    Serial.println(myICM.statusString());

    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.println("IMU ERROR!");

        while (true)
            delay(1000);
    }

    Serial.println("Initializing DMP...");

    // [FIX5]
    bool ok = true;                                         // '&=' so that a failed step is never overwritten by a later success
    ok &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    ok &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    ok &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    ok &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
    ok &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);
    ok &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    ok &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    ok &= (myICM.resetDMP() == ICM_20948_Stat_Ok);          // was missing; the SparkFun example does resetDMP() then resetFIFO()
    ok &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    if (!ok)
    {
        Serial.println("DMP INIT FAILED (is '#define ICM_20948_USE_DMP' enabled in ICM_20948_C.h?)");
        while (true)
            delay(1000);
    }

    const float an = sqrtf(AXIS_B[0]*AXIS_B[0] + AXIS_B[1]*AXIS_B[1] + AXIS_B[2]*AXIS_B[2]);
    if (an > 1e-6f)
    {
        for (int i = 0; i < 3; i++) axisN[i] = AXIS_B[i] / an;
        axisSet = true;
    }
    else
        Serial.println("AXIS_B not set: Twist/Swing are NOT computed. Use the w(dps) vector to find the shaft axis.");

    Serial.println("DMP READY. Keep the base still, wait for the DMP to settle (watch dQ), then send 'z'.");

    lastControlTime = micros();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // Ручное управление сервой и захват опорного положения через Serial
    while (Serial.available() > 0) {
    char c = Serial.read();

    // Захват опорного положения — команда z
    if (c == 'z' || c == 'Z') {
        refRequested = true;
        refQn = 0;
        refGn = 0;

        for (int i = 0; i < 3; i++) {
        refGacc[i] = 0.0f;
        }

        servoCommand = "";
        Serial.println("REF capture requested...");
        continue;
    }

    // Числовая команда серве завершается переводом строки
    if (c == '\n' || c == '\r') {
        if (servoCommand.length() > 0) {
        int pulse = servoCommand.toInt();

        if (pulse >= 1100 && pulse <= 1900) {
            myServo.writeMicroseconds(pulse);
            Serial.printf("AUTO: roll=%.1f pulse=%d axisSet=%d ref=%d\n",
              roll, pulse, axisSet, orientationInitialized);
            Serial.print("Servo pulse: ");
            Serial.println(pulse);
        } else {
            Serial.println("Enter 1100..1900");
        }

        servoCommand = "";
        }
    } else if (c >= '0' && c <= '9') {
        if (servoCommand.length() < 4) {
        servoCommand += c;
        } else {
        servoCommand = "";
        }
    } else {
        servoCommand = "";
    }
    }

    readDMP();

    uint32_t now = micros();

    if ((uint32_t)(now - lastControlTime) < CONTROL_PERIOD_US)
        return;

    lastControlTime = now;
    
    // Автоматическая стабилизация по углу вокруг оси Y
    if (axisSet && orientationInitialized)
    {
        const float KP_US_PER_DEG = 18.0f;
        const int SERVO_CENTER_US = 1500;
        const int MAX_CORRECTION_US = 350;

        float correction = -roll * KP_US_PER_DEG;
        correction = constrain(
            correction,
            (float)-MAX_CORRECTION_US,
            (float)MAX_CORRECTION_US
        );

        int pulse = SERVO_CENTER_US + (int)lroundf(correction);
        myServo.writeMicroseconds(pulse);
        Serial.printf("STAB: roll=%.1f pulse=%d axis=%d ref=%d\n",
            roll, pulse, axisSet, orientationInitialized);
    }


    if (lastDMPTime == 0 ||
        millis() - lastDMPTime > DMP_TIMEOUT_MS)
    {
        static uint32_t lastWarn = 0;                       // no delay(100) here: the FIFO must keep being drained
        if (millis() - lastWarn > 1000) { lastWarn = millis(); Serial.println("WARNING: DMP DATA TIMEOUT"); }
        return;
    }

    static uint32_t lastPrint = 0;

    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();

        // [FIX2/6] was "RollY" (twist about a hard-wired Y); now twist about AXIS_B, plus swing and the gyro cross-check
        Serial.print("Twist: ");
        if (axisSet && orientationInitialized) Serial.print(roll, 2); else Serial.print("n/a");

        Serial.print(" | Swing: ");
        if (axisSet && orientationInitialized) Serial.print(swingDeg, 1); else Serial.print("n/a");

        Serial.print(" | GyroInt: ");
        Serial.print(gyroIntDeg, 2);

        Serial.print(" | w(dps) X,Y,Z: ");
        Serial.print(gW[0], 1); Serial.print(",");
        Serial.print(gW[1], 1); Serial.print(",");
        Serial.print(gW[2], 1);

        Serial.print(" | GyroY: ");
        Serial.print(gyroRate, 1);

        Serial.print(" | GyroYFilt: ");
        Serial.print(gyroRateFiltered, 1);

        Serial.print(" | Q: ");
        Serial.print(q0, 4);
        Serial.print(", ");
        Serial.print(q1, 4);
        Serial.print(", ");
        Serial.print(q2, 4);
        Serial.print(", ");
        Serial.print(q3, 4);

        Serial.print(" | DMPcnt: ");
        Serial.print(dmpCounter);

        // how fast q still changes (deg/s): wait until it is ~0 before sending 'z'
        static float qMark[4] = { 1, 0, 0, 0 }; static uint32_t tMark = 0;
        if (millis() - tMark >= 1000)
        {
            const float cur[4] = { q0, q1, q2, q3 };
            Serial.print(" | dQ(deg/s): "); Serial.print(qAngleF(qMark, cur) * 180.0f / PI * 1000.0f / (float)(millis() - tMark), 2);
            for (int i = 0; i < 4; i++) qMark[i] = cur[i];
            tMark = millis();
        }
        Serial.print(" | err: "); Serial.print(statusErrors); Serial.print("/"); Serial.println(badNorm);
    }
}
