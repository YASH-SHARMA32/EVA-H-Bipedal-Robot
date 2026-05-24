/**
 * @file EVA_H_Dynamic_Gait_Engine.ino
 * @brief High-Degree-of-Freedom Bipedal Locomotion Core with Active IMU Balancing
 * @author YASH-SHARMA32
 * @version 6.0.0-PRO
 * * Abstract: Implements a multi-tasking FreeRTOS closed-loop locomotion engine. 
 * Integrates MPU6050 sensor fusion via a Complementary Filter for real-time 
 * active torque compensation and dynamic S-Curve gait generation across 8 DoF.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <esp_task_wdt.h>

// ====================================================================
// 1. SYSTEM CONFIGURATION & ACTUATOR HARDWARE MAPPING
// ====================================================================
#define SYSTEM_VERSION "6.0.0-PRO"
#define CONTROL_LOOP_PERIOD_MS 20  // Deterministic 50Hz control cycle
#define WATCHDOG_TIMEOUT_SEC   5

// Pin Assignments (Strictly matching System Blueprints)
#define ACTUATOR_LEFT_HIP_ROLL     13
#define ACTUATOR_LEFT_HIP_PITCH    12
#define ACTUATOR_LEFT_KNEE         14
#define ACTUATOR_LEFT_ANKLE        27
#define ACTUATOR_RIGHT_HIP_ROLL    26
#define ACTUATOR_RIGHT_HIP_PITCH   25
#define ACTUATOR_RIGHT_KNEE        33
#define ACTUATOR_RIGHT_ANKLE       32

// I2C Pins for MPU6050 IMU Sensor
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22

// ====================================================================
// 2. KINEMATIC GEOMETRY CONSTANTS
// ====================================================================
namespace Kinematics {
    constexpr float NEUTRAL               = 90.0f;
    constexpr float ROLL_WEIGHT_SHIFT_IN  = 84.0f;
    constexpr float ROLL_WEIGHT_SHIFT_OUT = 96.0f;
    constexpr float HIP_SWING_FWD         = 105.0f;
    constexpr float HIP_SWING_FWD_LEFT    = 105.0f;
    constexpr float HIP_TORQUE_COMP_BACK  = 75.0f;
    constexpr float KNEE_LIFT_ANGLE       = 65.0f;
    constexpr float KNEE_LAND_ANGLE       = 85.0f;
    constexpr float ANKLE_TOE_UP          = 80.0f;
    constexpr float ANKLE_SUPPORT_TILT    = 98.0f;
    
    // Safety Thresholds
    constexpr float CRITICAL_FALL_LIMIT   = 35.0f; // Force cut-off angle in degrees
}

// ====================================================================
// 3. SYSTEM STRUCTURES & DATA TYPES
// ====================================================================
enum class GaitPhase : uint8_t {
    PHASE_WEIGHT_SHIFT,
    PHASE_SWING_LIFT,
    PHASE_SWING_FORWARD,
    PHASE_TERMINAL_PLACE,
    PHASE_RESET_NEUTRAL
};

enum class ActiveLeg : uint8_t {
    LEG_RIGHT,
    LEG_LEFT
};

struct IMUData {
    float pitch = 0.0f;
    float roll  = 0.0f;
    bool isSystemStable = true;
};

struct KinematicMatrix {
    float currentAngles;
    float targetAngles;
    float startAngles;
    GaitPhase currentPhase;
    ActiveLeg activeLeg;
};

// Global Shared Thread-Safe Variables
KinematicMatrix robotGeometry;
IMUData telemetryPacket;
portMUX_TYPE dataMutex = portMUX_INITIALIZER_UNLOCKED;

// ====================================================================
// 4. SERVO DRIVER SYSTEM WITH MECHANICAL INVERSION LAYER
// ====================================================================
class AdvancedServoDriver {
private:
    Servo servoObjects;
    bool reversed;
    int pins;

public:
    AdvancedServoDriver() {
        pins = ACTUATOR_LEFT_HIP_ROLL;   reversed = false;
        pins = ACTUATOR_LEFT_HIP_PITCH;  reversed = true;  // Hardware inversion matrix
        pins = ACTUATOR_LEFT_KNEE;       reversed = true;  // Hardware inversion matrix
        pins = ACTUATOR_LEFT_ANKLE;      reversed = false;
        pins = ACTUATOR_RIGHT_HIP_ROLL;  reversed = false;
        pins = ACTUATOR_RIGHT_HIP_PITCH; reversed = false;
        pins = ACTUATOR_RIGHT_KNEE;      reversed = false;
        pins = ACTUATOR_RIGHT_ANKLE;     reversed = false;
    }

    void attachAll() {
        for (int i = 0; i < 8; i++) {
            servoObjects[i].attach(pins[i]);
            servoObjects[i].write(90);
        }
        delay(500);
    }

    void writeDirect(int idx, float angle) {
        float commandAngle = constrain(angle, 0.0f, 180.0f);
        if (reversed[idx]) commandAngle = 180.0f - commandAngle;
        servoObjects[idx].write((int)commandAngle);
    }

    void detachAll() {
        for(int i = 0; i < 8; i++) servoObjects[i].detach();
    }
};

// ====================================================================
// 5. CLOSED-LOOP DYNAMIC BALANCE & GAIT ENGINE (CORE 0)
// ====================================================================
class ClosedLoopGaitEngine {
private:
    AdvancedServoDriver& servos;
    uint32_t phaseDurationsMS = {1000, 800, 1000, 800, 1000};

    float computeSigmoidFactor(float t_normalized) {
        float k = 10.0f; // Curve steepness parameter
        float t_mid = 0.5f;
        float raw_sigma = 1.0f / (1.0f + exp(-k * (t_normalized - t_mid)));
        float sigma_0 = 1.0f / (1.0f + exp(-k * (0.0f - t_mid)));
        float sigma_1 = 1.0f / (1.0f + exp(-k * (1.0f - t_mid)));
        return (raw_sigma - sigma_0) / (sigma_1 - sigma_0);
    }

    void updateGaitTargetMatrix() {
        for (int i = 0; i < 8; i++) robotGeometry.targetAngles[i] = Kinematics::NEUTRAL;

        if (robotGeometry.activeLeg == ActiveLeg::LEG_RIGHT) {
            switch (robotGeometry.currentPhase) {
                case GaitPhase::PHASE_WEIGHT_SHIFT:
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_IN;
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_OUT;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    break;
                case GaitPhase::PHASE_SWING_LIFT:
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_IN;
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_OUT;
                    robotGeometry.targetAngles = Kinematics::KNEE_LIFT_ANGLE;
                    robotGeometry.targetAngles = Kinematics::ANKLE_TOE_UP;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    break;
                case GaitPhase::PHASE_SWING_FORWARD:
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_IN;
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_OUT;
                    robotGeometry.targetAngles = Kinematics::KNEE_LIFT_ANGLE;
                    robotGeometry.targetAngles = Kinematics::HIP_SWING_FWD;
                    robotGeometry.targetAngles = Kinematics::HIP_TORQUE_COMP_BACK;
                    robotGeometry.targetAngles = Kinematics::ANKLE_TOE_UP;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    break;
                case GaitPhase::PHASE_TERMINAL_PLACE:
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_IN;
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_OUT;
                    robotGeometry.targetAngles = Kinematics::KNEE_LAND_ANGLE;
                    robotGeometry.targetAngles = Kinematics::HIP_SWING_FWD;
                    robotGeometry.targetAngles = Kinematics::HIP_TORQUE_COMP_BACK;
                    robotGeometry.targetAngles = Kinematics::NEUTRAL;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    break;
                case GaitPhase::PHASE_RESET_NEUTRAL:
                    break;
            }
        } else {
            switch (robotGeometry.currentPhase) {
                case GaitPhase::PHASE_WEIGHT_SHIFT:
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_OUT;
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_IN;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    break;
                case GaitPhase::PHASE_SWING_LIFT:
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_OUT;
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_IN;
                    robotGeometry.targetAngles = Kinematics::KNEE_LIFT_ANGLE;
                    robotGeometry.targetAngles = Kinematics::ANKLE_TOE_UP;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    break;
                case GaitPhase::PHASE_SWING_FORWARD:
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_OUT;
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_IN;
                    robotGeometry.targetAngles = Kinematics::KNEE_LIFT_ANGLE;
                    robotGeometry.targetAngles = Kinematics::HIP_SWING_FWD_LEFT;
                    robotGeometry.targetAngles = Kinematics::HIP_TORQUE_COMP_BACK;
                    robotGeometry.targetAngles = Kinematics::ANKLE_TOE_UP;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    break;
                case GaitPhase::PHASE_TERMINAL_PLACE:
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_OUT;
                    robotGeometry.targetAngles = Kinematics::ROLL_WEIGHT_SHIFT_IN;
                    robotGeometry.targetAngles = Kinematics::KNEE_LAND_ANGLE;
                    robotGeometry.targetAngles = Kinematics::HIP_SWING_FWD_LEFT;
                    robotGeometry.targetAngles = Kinematics::HIP_TORQUE_COMP_BACK;
                    robotGeometry.targetAngles = Kinematics::NEUTRAL;
                    robotGeometry.targetAngles = Kinematics::ANKLE_SUPPORT_TILT;
                    break;
                case GaitPhase::PHASE_RESET_NEUTRAL:
                    break;
            }
        }
    }

public:
    ClosedLoopGaitEngine(AdvancedServoDriver& s) : servos(s) {
        for (int i = 0; i < 8; i++) {
            robotGeometry.currentAngles[i] = Kinematics::NEUTRAL;
            robotGeometry.targetAngles[i] = Kinematics::NEUTRAL;
            robotGeometry.startAngles[i] = Kinematics::NEUTRAL;
        }
        robotGeometry.currentPhase = GaitPhase::PHASE_WEIGHT_SHIFT;
        robotGeometry.activeLeg = ActiveLeg::LEG_RIGHT;
    }

    void initializeIMU() {
        Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 400000);
        Wire.beginTransmission(0x68); // MPU6050 Address
        Wire.write(0x6B);             // PWR_MGMT_1 Register
        Wire.write(0);                // Wake up IMU
        Wire.endTransmission(true);
        Serial.println("[IMU] MPU6050 Sensor Online & Configured at 400000Hz Fast Mode.");
    }

    void readHardwareIMU(float& outPitch, float& outRoll) {
        Wire.beginTransmission(0x68);
        Wire.write(0x3B); // Starting register for Accelerometer Data
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 14, true);

        int16_t acX = Wire.read() << 8 | Wire.read();
        int16_t acY = Wire.read() << 8 | Wire.read();
        int16_t acZ = Wire.read() << 8 | Wire.read();
        Wire.read() << 8 | Wire.read(); // Skip temperature bytes
        int16_t gyX = Wire.read() << 8 | Wire.read();
        int16_t gyY = Wire.read() << 8 | Wire.read();

        // High-level Sensor Fusion Filter Math Matrix Calculations
        float accPitch = atan2((float)acY, (float)acZ) * 57.2957f;
        float accRoll  = atan2((float)-acX, sqrt((float)acY*acY + (float)acZ*acZ)) * 57.2957f;

        float gyroPitchRate = (float)gyX / 131.0f;
        float gyroRollRate  = (float)gyY / 131.0f;

        // Complementary Filter Algorithm Execution Layer
        static float lastFlippedPitch = 0;
        static float lastFlippedRoll  = 0;
        float dt = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;

        outPitch = 0.96f * (lastFlippedPitch + gyroPitchRate * dt) + 0.04f * accPitch;
        outRoll  = 0.96f * (lastFlippedRoll + gyroRollRate * dt) + 0.04f * accRoll;

        lastFlippedPitch = outPitch;
        lastFlippedRoll  = outRoll;
    }

    void executeThreadCore() {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        uint32_t msElapsed = 0;
        initializeIMU();
        updateGaitTargetMatrix();

        while (true) {
            float pitchError = 0.0f, rollError = 0.0f;
            readHardwareIMU(pitchError, rollError);

            // Thread Safety Layer Lock
            portENTER_CRITICAL(&dataMutex);
            telemetryPacket.pitch = pitchError;
            telemetryPacket.roll = rollError;
            
            // Check Emergency Anti-Destruct Interlock Status
            if (abs(pitchError) > Kinematics::CRITICAL_FALL_LIMIT || abs(rollError) > Kinematics::CRITICAL_FALL_LIMIT) {
                telemetryPacket.isSystemStable = false;
            }
            portEXIT_CRITICAL(&dataMutex);

            if (!telemetryPacket.isSystemStable) {
                servos.detachAll(); // Cut power to actuators immediately
                Serial.println("⛔ [CRITICAL EMERGENCY INTERLOCK ACTIVATED] Fall Limit Exceeded. Actuators Detached.");
                while(true) vTaskDelay(pdMS_TO_TICKS(1000));
            }

            uint32_t currentPhaseLimit = phaseDurationsMS[(uint8_t)robotGeometry.currentPhase];
            float progress = (float)msElapsed / (float)currentPhaseLimit;
            if (progress > 1.0f) progress = 1.0f;
            float trajectoryScale = computeSigmoidFactor(progress);

            // Dynamic Proportional Torque Injection Calculation
            float rollBalanceModifier  = rollError * 0.35f;  // Proportional Tuning Gain
            float pitchBalanceModifier = pitchError * 0.20f; // Proportional Tuning Gain

            for (int i = 0; i < 8; i++) {
                float nominalBaseAngle = robotGeometry.startAngles[i] + (robotGeometry.targetAngles[i] - robotGeometry.startAngles[i]) * trajectoryScale;
                
                // Closed-Loop Active Feedback Mapping Layer
                if (i == 0 || i == 4) nominalBaseAngle += rollBalanceModifier;   // Hip Roll Joint Injection Matrix
                if (i == 3 || i == 7) nominalBaseAngle -= rollBalanceModifier;   // Ankle Roll Joint Injection Matrix
                if (i == 1 || i == 5) nominalBaseAngle += pitchBalanceModifier;  // Hip Pitch Joint Injection Matrix

                robotGeometry.currentAngles[i] = nominalBaseAngle;
                servos.writeDirect(i, nominalBaseAngle);
            }

            msElapsed += CONTROL_LOOP_PERIOD_MS;

            if (msElapsed >= currentPhaseLimit) {
                robotGeometry.currentPhase = (GaitPhase)(((uint8_t)robotGeometry.currentPhase + 1) % 5);
                msElapsed = 0;

                if (robotGeometry.currentPhase == GaitPhase::PHASE_WEIGHT_SHIFT) {
                    robotGeometry.activeLeg = (robotGeometry.activeLeg == ActiveLeg::LEG_RIGHT) ? ActiveLeg::LEG_LEFT : ActiveLeg::LEG_RIGHT;
                }
                for (int i = 0; i < 8; i++) robotGeometry.startAngles[i] = robotGeometry.currentAngles[i];
                updateGaitTargetMatrix();
            }

            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
        }
    }
};

// ====================================================================
// 6. ASYNCHRONOUS TELEMETRY HUB TASK LAYER (CORE 1)
// ====================================================================
void vTelemetryCoreWorker(void *pvParameters) {
    while(true) {
        portENTER_CRITICAL(&dataMutex);
        float p = telemetryPacket.pitch;
        float r = telemetryPacket.roll;
        int stateID = (int)robotGeometry.currentPhase;
        const char* legStr = (robotGeometry.activeLeg == ActiveLeg::LEG_RIGHT) ? "RIGHT" : "LEFT";
        portEXIT_CRITICAL(&dataMutex);

        // Standardized Multi-Axis Kinematic Vector Matrix Log Formatting
        Serial.printf("[EVA-H TELEMETRY] Core 1 Stream -> Active Leg: %s | Phase ID: %d | Filtered Pitch: %.2f° | Filtered Roll: %.2f°\n", 
                      legStr, stateID, p, r);

        vTaskDelay(pdMS_TO_TICKS(250)); // Non-blocking asynchronous update rate (4Hz Telemetry Output)
    }
}

// ====================================================================
// 7. SYSTEM ENTRY & INITIALIZATION PROTOCOL
// ====================================================================
AdvancedServoDriver servoDriver;
ClosedLoopGaitEngine gaitEngine(servoDriver);

void vKinematicsTaskWorker(void *pvParameters) {
    gaitEngine.executeThreadCore();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔══════════════════════════════════════════════════════════╗");
    Serial.printf("║     EVA-H CLOSED-LOOP SENSOR FUSED GAIT CORE v%s    ║\n", SYSTEM_VERSION);
    Serial.println("║          Asynchronous Dual-Core Firmware Active          ║");
    Serial.println("╚══════════════════════════════════════════════════════════╝\n");

    servoDriver.attachAll();

    // Spawn Core 0 Task - Hard-Realtime Motion Calculus and Sensor Fusion Loop
    xTaskCreatePinnedToCore(
        vKinematicsTaskWorker, "GaitCore0", 4096, NULL, 3, NULL, 0
    );

    // Spawn Core 1 Task - Telemetry Processing Thread Layer
    xTaskCreatePinnedToCore(
        vTelemetryCoreWorker, "TelemetryCore1", 2048, NULL, 1, NULL, 1
    );

    esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL);
    Serial.println("[SYSTEM] Dual-core task architecture running safely.");
}

void loop() {
    // Left empty intentionally. Main Arduino thread sleeps while hardware cores execute dynamically.
    vTaskDelay(pdMS_TO_TICKS(1000));
}