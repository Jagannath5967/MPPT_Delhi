#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <driver/mcpwm.h>
#include <Adafruit_INA260.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_TSL2591.h>

// Standalone Vmp predictor — no TFLite library required (observation only, never modifies hardware)
#include "vmp_inference.h"

// ---- Access Point credentials ----
static const char *AP_SSID     = "MPPT-Dashboard";
static const char *AP_PASSWORD = "mppt1234";  // must be 8+ chars, or "" for open network

// Static AP IP configuration
static const IPAddress AP_LOCAL_IP(192, 168, 2, 5);
static const IPAddress AP_GATEWAY(192, 168, 2, 5);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

// ---- INA260 I2C sensor addresses ----
// INA260 default address = 0x40 (A0 and A1 both to GND).
// To set 0x41: connect A0 to VS (VCC). Verify with an I2C scan before use.
// Change these two constants to match the actual A0/A1 pin wiring on your boards.
constexpr uint8_t INA260_PV_ADDRESS      = 0x40;  // PV / solar side
constexpr uint8_t INA260_BATTERY_ADDRESS = 0x41;  // battery side

// Current sign correction.
// Set either to -1 if that INA260's IN+/IN- shunt leads are physically reversed,
// causing the sensor to report negative current when conventional positive current
// should flow. All other sign inversions are intentional and must not be hidden.
constexpr int8_t INA260_PV_CURRENT_SIGN      =  1;
constexpr int8_t INA260_BATTERY_CURRENT_SIGN =  1;

// I2C pins
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

Adafruit_INA260  ina260Pv;
Adafruit_INA260  ina260Batt;
bool pvSensorOk   = false;
bool battSensorOk = false;

constexpr uint8_t TMP117_ADDRESS  = 0x48;
// TSL2591 address is hardwired at 0x29; begin() takes no address parameter.

// ---- 10K NTC battery temperature sensor (GPIO34, ADC1) ----
constexpr uint8_t NTC_PIN         = 34;
constexpr float   NTC_VCC         = 3.3f;
constexpr float   NTC_ADC_MAX     = 4095.0f;   // 12-bit ESP32
constexpr float   NTC_R_FIXED     = 10000.0f;  // series resistor (VCC side)
constexpr float   NTC_R_NOMINAL   = 10000.0f;  // NTC resistance at 25°C
constexpr float   NTC_T_NOMINAL_C = 25.0f;
constexpr float   NTC_BETA        = 3950.0f;   // confirm with your NTC datasheet
constexpr float   MAX_BATTERY_TEMP_C = 45.0f;  // stop charging above this

Adafruit_TMP117  tmp117;
Adafruit_TSL2591 tsl2591(2591);
bool tmpSensorOk = false;
bool tslSensorOk = false;

float ambientTempC      = 0.0f;
float luxValue          = 0.0f;
float solarIrradianceWm2= 0.0f;
constexpr float LUX_TO_IRRADIANCE = 120.0f;  // W/m² per lux, standard daylight conversion
float batteryTempC  = 0.0f;
bool  ntcValid      = false;
bool  cloudyCondition  = false;

// HYB-AIMLSL per-zone enable flags (all on by default)
bool hybZoneFarEnabled  = true;
bool hybZoneNearEnabled = true;
bool hybZoneHoldEnabled = true;

constexpr unsigned long AUX_SAMPLE_INTERVAL_MS = 2000;

// ---- Cloudy condition detector ----
// Detects a passing cloud by relative drop from a live sunny baseline.
// Both PV power and irradiance (TSL2591) contribute via OR logic.
constexpr float         CLOUDY_PV_DROP_FRAC   = 0.35f;  // 35% drop in PV power  → triggers
constexpr float         CLOUDY_IRR_DROP_FRAC  = 0.35f;  // 35% drop in irradiance → triggers
constexpr float         CLOUDY_BASELINE_ALPHA = 0.05f;  // EMA rate for baseline (~40 s to settle)
constexpr float         CLOUDY_MIN_PV_W       = 3.0f;   // panel must produce ≥3 W for detection
constexpr float         CLOUDY_MIN_IRR_WM2    = 50.0f;  // irradiance must be ≥50 W/m² for detection
constexpr unsigned long CLOUDY_CONFIRM_MS     = 5000;   // drop must persist this long to confirm
constexpr unsigned long CLOUDY_CLEAR_MS       = 5000;   // recovery must persist this long to clear

// Sensor output validation bounds
constexpr float SENSOR_MAX_VOLTAGE_V = 40.0f;   // headroom over 3S LiHV full (13.05V) and panel Voc (22V)
constexpr float SENSOR_MAX_CURRENT_A = 15.0f;   // INA260 hardware maximum
constexpr float SENSOR_MAX_POWER_W   = 400.0f;  // realistic panel limit

constexpr unsigned long SENSOR_SAMPLE_INTERVAL_MS = 200;

// ---- Sensor EMA filtering ----
constexpr float PV_VOLTAGE_FILTER_ALPHA   = 0.20f;  // moderate -- PV can change quickly
constexpr float BATT_VOLTAGE_FILTER_ALPHA = 0.10f;  // slow/stable -- termination logic is sensitive
constexpr float CURRENT_FILTER_ALPHA      = 0.20f;
constexpr float POWER_FILTER_ALPHA        = 0.20f;
// Dead-band threshold: clamp |current| < this to 0 (symmetric noise gate).
// Does NOT hide actual negative current -- a real reverse current above this threshold
// will be reported with its correct sign.
constexpr float CURRENT_DEADBAND_A = 0.02f;

// ---- Synchronous buck PWM (IR2101 half-bridge driver) ----
constexpr uint8_t  PWM_HIGH_SIDE_PIN  = 18;  // IR2101 HIN
constexpr uint8_t  PWM_LOW_SIDE_PIN   = 19;  // IR2101 LIN
constexpr uint32_t PWM_FREQUENCY_HZ   = 20000;
constexpr uint32_t PWM_DEADTIME_TICKS = 5;   // 5 × 100ns = 500ns

// ---- MPPT Perturb & Observe ----
constexpr float MPPT_DUTY_STEP_PERCENT  = 0.25f;
constexpr float MPPT_DUTY_START_PERCENT = 10.0f;
constexpr float MPPT_DUTY_MIN_PERCENT   = 20.0f;
constexpr float MPPT_DUTY_MAX_PERCENT   = 85.0f;
constexpr unsigned long MPPT_STEP_INTERVAL_MS = 500;  // TODO: tune against output LC settling time
// Separate headroom thresholds (Victron-style):
// START: PV must exceed battery target + 5V before switching begins.
// RUNNING: PV only needs to exceed battery target + 1V to keep running.
// This prevents unnecessary stops as sunlight weakens during the day.
constexpr float MPPT_MIN_PV_HEADROOM_START_V   = 5.0f;
constexpr float MPPT_MIN_PV_HEADROOM_RUNNING_V = 1.0f;

// Absolute hard duty-cycle limits enforced in pwmSetDuty().
constexpr float PWM_DUTY_HARD_MIN_PERCENT = 10.0f;
constexpr float PWM_DUTY_HARD_MAX_PERCENT = 85.0f;

// ---- Panel datasheet (feedforward starting-duty estimate) ----
constexpr float PANEL_VOC_RATED  = 21.98f;
constexpr float PANEL_VMP_RATED  = 19.10f;
constexpr float PANEL_FOCV_RATIO = PANEL_VMP_RATED / PANEL_VOC_RATED;  // k ~0.869

// ---- Battery chemistry + cell count presets ----
constexpr int BATTERY_CELL_COUNT_MIN = 2;
constexpr int BATTERY_CELL_COUNT_MAX = 3;
int selectedCellCount = 3;

enum BatteryChemistry {
  CHEM_LIPO_STANDARD = 0,  // 4.20V/cell
  CHEM_LIPO_HV       = 1,  // 4.35V/cell ("LiHV")
  CHEM_LIFEPO4       = 2,  // 3.65V/cell
};
constexpr float CHEMISTRY_FULL_CELL_VOLTAGE[3]  = {4.20f, 4.35f, 3.65f};
constexpr float CHEMISTRY_EMPTY_CELL_VOLTAGE[3] = {3.00f, 3.00f, 2.80f};  // cutoff per cell
constexpr const char *CHEMISTRY_NAMES[3] = {
  "LiPo/Li-ion (4.20V/cell)", "LiPo HV (4.35V/cell)", "LiFePO4 (3.65V/cell)"
};
BatteryChemistry selectedChemistry = CHEM_LIPO_STANDARD;

float batteryFullVoltage() {
  return CHEMISTRY_FULL_CELL_VOLTAGE[selectedChemistry] * selectedCellCount;
}

float batteryEmptyVoltage() {
  return CHEMISTRY_EMPTY_CELL_VOLTAGE[selectedChemistry] * selectedCellCount;
}

// ---- Pack physical specs ----
constexpr float BATTERY_CAPACITY_AH   = 4.2f;
constexpr float MAX_CHARGE_CURRENT_A  = 2.0f;
constexpr float TERMINATION_CURRENT_A = BATTERY_CAPACITY_AH * 0.05f;  // C/20 = 0.21A
// Guard against false-termination during EMA settling at startup.
constexpr unsigned long MIN_CHARGE_RUNTIME_BEFORE_TERMINATION_MS = 30000;

// ---- Bulk-stage voltage ramp ----
constexpr unsigned long RAMP_DURATION_MS = 45000;  // TODO: confirm pacing preference
constexpr float CHARGE_CONTROL_CORRECTION_STEP_PERCENT = 0.5f;
constexpr float BATTERY_FULL_TOLERANCE_V = 0.05f;

// STAGE_RAMPING: rampCeilingVoltage still climbing (anti-inrush pacing, ~45s).
// STAGE_BULK:    ceiling reached target but actual battery voltage still below it.
// STAGE_HOLDING: actual battery voltage is near target -- true CV phase.
enum ChargeStage { STAGE_STOPPED, STAGE_RAMPING, STAGE_BULK, STAGE_HOLDING, STAGE_FLOAT };
ChargeStage chargeStage = STAGE_STOPPED;
float rampCeilingVoltage = 0.0f;
float rampStepPerCycle   = 0.0f;

const char *chargeStageStr() {
  switch (chargeStage) {
    case STAGE_RAMPING: return "Ramping";
    case STAGE_BULK:    return "Bulk";
    case STAGE_HOLDING: return "Holding (CV)";
    case STAGE_FLOAT:   return "Float (CV)";
    default:            return "Stopped";
  }
}

// ---- Stop type — controls auto-restart eligibility ----
enum StopType {
  STOP_USER,      // user pressed Stop → no auto-restart
  STOP_FAULT,     // sensor/thermal fault → no auto-restart
  STOP_COMPLETE,  // battery full → no auto-restart
  STOP_AUTO_PV,   // PV voltage dropped below running headroom → eligible for auto-restart
};
constexpr unsigned long AUTO_RESTART_INTERVAL_MS = 30000;  // 30s between re-check attempts

// ---- Operating mode ----
// MODE_MPPT:     Adaptive P&O — variable step size + full CC-CV stage machine (Bulk → Absorption → Float).
// MODE_HYBRID:   P&O guided by AI Vmp prediction — biased direction, variable step, convergence hold.
// MODE_PWM_AUTO: Commercial CC-CV charger — Bulk (CC) → Absorption (CV) → Float (LiFePO4 only).
enum OperatingMode { MODE_MPPT, MODE_HYBRID, MODE_PWM_AUTO };

// ---- Auto PWM (CC-CV) constants ----
constexpr float PWMCC_KP_CURRENT     = 0.25f;  // proportional gain: duty %  per Amp error
constexpr float PWMCV_KP_VOLTAGE     = 0.30f;  // proportional gain: duty % per Volt error
constexpr float PWMCC_MAX_STEP       = 0.5f;   // max duty correction per step (% pts)
constexpr float PWM_TEMP_COMP_MV_CELL = 3.0f;  // mV/°C/cell temperature compensation for absorption
constexpr float PWM_LIFEPO4_FLOAT_V_CELL = 3.35f;  // float target per cell (LiFePO4 only)
constexpr float PWM_REBULK_HYST_V    = 0.30f;  // voltage drop below float to re-enter bulk

// ---- Hybrid MPPT-AI constants ----
// Far band  (|duty - AI_target| > NEAR_BAND): AI overrides P&O direction, large step.
// Near band (HOLD_BAND < gap <= NEAR_BAND)  : P&O governs direction, fine step.
// Hold band (gap <= HOLD_BAND + power stable): skip perturbation, hold duty.
constexpr float HYBRID_STEP_FAR_PERCENT  = 0.50f;
constexpr float HYBRID_STEP_NEAR_PERCENT = 0.10f;
constexpr float HYBRID_NEAR_BAND_PERCENT = 2.0f;
constexpr float HYBRID_HOLD_BAND_PERCENT = 0.5f;
constexpr float HYBRID_HOLD_POWER_W      = 0.5f;  // max |ΔP| to allow hold


constexpr float CURRENT_FLOW_DETECT_A = 0.1f; // minimum current to flag hasSeenCurrentFlow

WebServer server(80);

// Filtered measurement outputs consumed by CC/CV/MPPT
float pvVoltage      = 0.0f;
float pvCurrent      = 0.0f;
float pvPower        = 0.0f;   // from INA260 native power register
float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
float batteryPower   = 0.0f;   // from INA260 native power register


// Pre-filter values exposed on the dashboard sensor debug card
float rawPvVoltage   = 0.0f;
float rawBattVoltage = 0.0f;
float rawPvCurrent   = 0.0f;
float rawBattCurrent = 0.0f;

float dutyPercent = MPPT_DUTY_START_PERCENT;
float previousPvPower = 0.0f;
int8_t perturbDirection = 1;  // +1 = increasing duty, -1 = decreasing
unsigned long lastMpptStep = 0;
bool mpptRunning = false;
OperatingMode operatingMode = MODE_MPPT;
uint32_t perturbCount = 0;  // increments every control step; dashboard uses it to detect new perturbations
String lastStopReason = "Not started yet";
StopType lastStopType       = STOP_USER;
bool     autoRestartEnabled = false;
unsigned long lastAutoStopTime = 0;

// ---- Vmp predictor (observation only — never modifies PWM/P&O) ----
// Model: Input(14)->Dense(32)->BN->ReLU->Dense(16)->BN->ReLU->Dense(8)->BN->ReLU->Dense(1,linear)
// Weights and scalers embedded in vmp_inference.h — no TFLite runtime required.

bool    regValid              = false;  // inference ran successfully this step
float   regPredVmpV           = 0.0f;   // predicted Vmp (V) — observation only, never written to PWM
float   regPredDutyPct        = 0.0f;   // feedforward duty derived from regPredVmpV — display only
bool    vmpTransientRecovery  = false;  // set for one tick after transient clears — triggers P&O re-seed

// ---- Energy & efficiency measurement ----
constexpr float ENERGY_MIN_POWER_W = 0.5f;  // min PV power for valid efficiency calculation
float pvEnergyWh          = 0.0f;
float batteryEnergyWh     = 0.0f;
float pvPowerPrev         = 0.0f;
float battPowerPrev       = 0.0f;
unsigned long lastEnergyTime = 0;
float instantEfficiency    = 0.0f;
float cumulativeEfficiency = 0.0f;
bool  efficiencyValid      = false;
bool  effSuspicious        = false;
uint32_t sessionId         = 0;
float lastFeedforwardDutyPercent = 0.0f;
float lastVmpEstimate            = 0.0f;
unsigned long chargingStartTime = 0;
bool hasSeenCurrentFlow = false;  // true once batteryCurrent > 0.1A this session

// Forward declaration -- serviceSensors() calls stopCharging() which is defined below.
void stopCharging(const String &reason, StopType stopType = STOP_USER);

// --- Sensor output validators ---
static inline bool isValidVoltage(float v) {
  return !isnan(v) && !isinf(v) && v >= 0.0f && v <= SENSOR_MAX_VOLTAGE_V;
}
static inline bool isValidCurrent(float i) {
  return !isnan(i) && !isinf(i) && i >= -SENSOR_MAX_CURRENT_A && i <= SENSOR_MAX_CURRENT_A;
}
static inline bool isValidPower(float p) {
  return !isnan(p) && !isinf(p) && p >= -SENSOR_MAX_POWER_W && p <= SENSOR_MAX_POWER_W;
}

// Recover I2C bus if a previous timeout left SCL/SDA in a mid-transaction state.
// Pings the TMP117 address; if it gets no ACK, reinits Wire to clear the lockup.
static void recoverI2C(uint8_t probeAddr = TMP117_ADDRESS) {
  Wire.beginTransmission(probeAddr);
  if (Wire.endTransmission() != 0) {
    Wire.end();
    delay(2);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(50);
  }
}

// sampleAuxSensors: read TMP117 (temperature) and TSL2591 (light) at a reduced
// rate (AUX_SAMPLE_INTERVAL_MS = 2s).  These are non-critical for the control
// loop so they do NOT trigger a charge-stop on failure.
void sampleAuxSensors() {
  if (tmpSensorOk) {
    recoverI2C();
    sensors_event_t tempEvent;
    tmp117.getEvent(&tempEvent);
    float t = tempEvent.temperature;
    if (!isnan(t) && t > -40.0f && t < 125.0f) {
      ambientTempC = t;
    }
  }
  if (tslSensorOk) {
    recoverI2C(0x29);
    uint32_t lum  = tsl2591.getFullLuminosity();
    uint16_t ir   = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    float lux     = tsl2591.calculateLux(full, ir);
    if (!isnan(lux) && !isinf(lux) && lux >= 0.0f && lux < 200000.0f) {
      luxValue           = lux;
      solarIrradianceWm2 = lux / LUX_TO_IRRADIANCE;
    }
  }

  // ---- NTC battery temperature (GPIO34, ADC1) ----
  // Circuit: 3.3V → R_FIXED(10K) → GPIO34 → NTC(10K) → GND
  // Average 16 samples; no delay needed -- ESP32 ADC settles in <1µs.
  {
    uint32_t adcSum = 0;
    for (int i = 0; i < 16; i++) adcSum += analogRead(NTC_PIN);
    float adcVal = adcSum / 16.0f;
    float voltage = adcVal * NTC_VCC / NTC_ADC_MAX;

    if (voltage > 0.05f && voltage < NTC_VCC - 0.05f) {
      float rNtc = NTC_R_FIXED * voltage / (NTC_VCC - voltage);
      float tK   = 1.0f / ((1.0f / (NTC_T_NOMINAL_C + 273.15f)) +
                            (1.0f / NTC_BETA) * logf(rNtc / NTC_R_NOMINAL));
      float tC   = tK - 273.15f;
      if (tC > -40.0f && tC < 125.0f) {
        batteryTempC = tC;
        ntcValid     = true;
      }
    } else {
      ntcValid = false;  // open circuit or short
    }
  }
}

void resetEnergySession() {
  pvEnergyWh           = 0.0f;
  batteryEnergyWh      = 0.0f;
  pvPowerPrev          = pvPower;
  battPowerPrev        = batteryPower;
  lastEnergyTime       = millis();
  instantEfficiency    = 0.0f;
  cumulativeEfficiency = 0.0f;
  efficiencyValid      = false;
  effSuspicious        = false;
  sessionId++;
}

// Trapezoidal energy integration and efficiency calculation.
// Called every SENSOR_SAMPLE_INTERVAL_MS (200ms), after sampleSensors().
// Does not modify any MPPT or sensor state.
void updateEnergy() {
  unsigned long now = millis();
  float dtSec = (now - lastEnergyTime) / 1000.0f;
  lastEnergyTime = now;

  if (!mpptRunning || !pvSensorOk || !battSensorOk) {
    pvPowerPrev   = pvPower;
    battPowerPrev = batteryPower;
    return;
  }

  bool pvOk   = isValidPower(pvPower)      && pvPower >= 0.0f;
  bool battOk = isValidPower(batteryPower) && batteryPower >= 0.0f;

  // Trapezoidal rule: E += 0.5*(P_prev + P_now)*dt/3600
  // Guard dt: skip integration if sample is stale (>5s) or zero
  if (pvOk   && dtSec > 0.0f && dtSec < 5.0f)
    pvEnergyWh      += 0.5f * (pvPowerPrev   + pvPower)      * dtSec / 3600.0f;
  if (battOk && dtSec > 0.0f && dtSec < 5.0f)
    batteryEnergyWh += 0.5f * (battPowerPrev + batteryPower) * dtSec / 3600.0f;

  if (pvOk)   pvPowerPrev   = pvPower;
  if (battOk) battPowerPrev = batteryPower;

  // Instantaneous efficiency — only when PV power is above noise floor
  effSuspicious = false;
  if (pvOk && battOk && pvPower >= ENERGY_MIN_POWER_W) {
    float ie = (batteryPower / pvPower) * 100.0f;
    if (ie > 105.0f) {
      // Over 105% is physically impossible — flag as suspicious, do not use
      effSuspicious   = true;
      efficiencyValid = false;
      Serial.printf("WARN: Suspicious efficiency %.1f%% (PV=%.2fW Bat=%.2fW) -- ignored\n",
                    ie, pvPower, batteryPower);
    } else {
      instantEfficiency = ie;
      efficiencyValid   = true;
    }
  } else {
    efficiencyValid = false;
  }

  // Cumulative energy efficiency
  cumulativeEfficiency = (pvEnergyWh > 0.001f)
    ? (batteryEnergyWh / pvEnergyWh) * 100.0f
    : 0.0f;
}

// sampleSensors: read both INA260s, validate every reading, apply EMA.
// Called from serviceSensors() every SENSOR_SAMPLE_INTERVAL_MS.
// No delay() here -- I2C reads return in <2ms per channel.
void sampleSensors() {
  // ---- PV sensor (INA260 #1) ----
  static float lastGoodPvVolt = 0.0f;
  static float lastGoodPvCurr = 0.0f;
  static float lastGoodPvPow  = 0.0f;

  float rawPvVolt = 0.0f, rawPvCurr = 0.0f, rawPvPow = 0.0f;
  if (pvSensorOk) {
    float v = ina260Pv.readBusVoltage() / 1000.0f;                        // mV → V
    float i = ina260Pv.readCurrent()    / 1000.0f * INA260_PV_CURRENT_SIGN; // mA → A, sign-corrected
    float p = ina260Pv.readPower()      / 1000.0f;                        // mW → W

    if (isValidVoltage(v) && isValidCurrent(i) && isValidPower(p)) {
      rawPvVolt = v;  rawPvCurr = i;  rawPvPow = p;
      lastGoodPvVolt = v;  lastGoodPvCurr = i;  lastGoodPvPow = p;
    } else {
      // Corrupt read -- hold last good so the control loop does not see a spike
      rawPvVolt = lastGoodPvVolt;
      rawPvCurr = lastGoodPvCurr;
      rawPvPow  = lastGoodPvPow;
      Serial.println("WARN: PV INA260 returned invalid reading -- holding last good");
    }
  }

  // ---- Battery sensor (INA260 #2) ----
  static float lastGoodBattVolt = 0.0f;
  static float lastGoodBattCurr = 0.0f;
  static float lastGoodBattPow  = 0.0f;

  float rawBattVolt = 0.0f, rawBattCurr = 0.0f, rawBattPow = 0.0f;
  if (battSensorOk) {
    float v = ina260Batt.readBusVoltage() / 1000.0f;
    float i = ina260Batt.readCurrent()    / 1000.0f * INA260_BATTERY_CURRENT_SIGN;
    float p = ina260Batt.readPower()      / 1000.0f;

    if (isValidVoltage(v) && isValidCurrent(i) && isValidPower(p)) {
      rawBattVolt = v;  rawBattCurr = i;  rawBattPow = p;
      lastGoodBattVolt = v;  lastGoodBattCurr = i;  lastGoodBattPow = p;
    } else {
      rawBattVolt = lastGoodBattVolt;
      rawBattCurr = lastGoodBattCurr;
      rawBattPow  = lastGoodBattPow;
      Serial.println("WARN: Battery INA260 returned invalid reading -- holding last good");
    }
  }

  // Store pre-filter values for dashboard diagnostics
  rawPvVoltage   = rawPvVolt;
  rawBattVoltage = rawBattVolt;
  rawPvCurrent   = rawPvCurr;
  rawBattCurrent = rawBattCurr;

  // ---- EMA low-pass filter ----
  static float pvVoltFiltered   = 0.0f;
  static float battVoltFiltered = 0.0f;
  static float pvCurrFiltered   = 0.0f;
  static float battCurrFiltered = 0.0f;
  static float pvPowFiltered    = 0.0f;
  static float battPowFiltered  = 0.0f;
  static bool  emaInitialised   = false;

  if (!emaInitialised) {
    // Seed from first real reading -- prevents the 0→actual ramp that would cause
    // wrong rampStepPerCycle or a false STAGE_HOLDING trigger at startup.
    pvVoltFiltered   = rawPvVolt;
    battVoltFiltered = rawBattVolt;
    pvCurrFiltered   = rawPvCurr;
    battCurrFiltered = rawBattCurr;
    pvPowFiltered    = rawPvPow;
    battPowFiltered  = rawBattPow;
    emaInitialised   = true;
  } else {
    pvVoltFiltered   = PV_VOLTAGE_FILTER_ALPHA   * rawPvVolt   + (1.0f - PV_VOLTAGE_FILTER_ALPHA)   * pvVoltFiltered;
    battVoltFiltered = BATT_VOLTAGE_FILTER_ALPHA * rawBattVolt + (1.0f - BATT_VOLTAGE_FILTER_ALPHA) * battVoltFiltered;
    pvCurrFiltered   = CURRENT_FILTER_ALPHA      * rawPvCurr   + (1.0f - CURRENT_FILTER_ALPHA)      * pvCurrFiltered;
    battCurrFiltered = CURRENT_FILTER_ALPHA      * rawBattCurr + (1.0f - CURRENT_FILTER_ALPHA)      * battCurrFiltered;
    pvPowFiltered    = POWER_FILTER_ALPHA        * rawPvPow    + (1.0f - POWER_FILTER_ALPHA)        * pvPowFiltered;
    battPowFiltered  = POWER_FILTER_ALPHA        * rawBattPow  + (1.0f - POWER_FILTER_ALPHA)        * battPowFiltered;
  }

  // Dead-band: symmetric noise gate -- clamps |x| < threshold to 0.
  // Actual negative current above the threshold is preserved with its sign.
  if (fabsf(pvCurrFiltered)   < CURRENT_DEADBAND_A) pvCurrFiltered   = 0.0f;
  if (fabsf(battCurrFiltered) < CURRENT_DEADBAND_A) battCurrFiltered = 0.0f;

  pvVoltage      = pvVoltFiltered;
  batteryVoltage = battVoltFiltered;
  pvCurrent      = pvCurrFiltered;
  batteryCurrent = battCurrFiltered;
  pvPower        = pvPowFiltered;
  batteryPower   = battPowFiltered;
}

// Dual-sensor cloud detector using relative drop from a live sunny baseline.
//
// Baseline: slow EMA of pvPower and irradiance, updated only while confirmed clear
// and charger is running. Freezes when cloudy so a long cloud doesn't erode the reference.
//
// Trigger (OR logic): either PV power OR irradiance drops >35% below its baseline.
// This handles TSL2591-absent hardware gracefully — pvPower alone is sufficient.
//
// Hysteresis: drop must persist CLOUDY_CONFIRM_MS; recovery must persist CLOUDY_CLEAR_MS.
void updateCloudyDetector() {
  static float         cloudPvBaseline  = 0.0f;
  static float         cloudIrrBaseline = 0.0f;
  static unsigned long cloudyStartMs = 0;
  static unsigned long clearStartMs  = 0;
  unsigned long now = millis();

  if (!mpptRunning) {
    cloudyStartMs = 0;
    clearStartMs  = 0;
    return;
  }

  // Update baselines only while confirmed clear, so a prolonged cloud doesn't drift them down.
  if (!cloudyCondition) {
    if (pvPower > CLOUDY_MIN_PV_W)
      cloudPvBaseline = cloudPvBaseline * (1.0f - CLOUDY_BASELINE_ALPHA)
                        + pvPower * CLOUDY_BASELINE_ALPHA;
    if (tslSensorOk && solarIrradianceWm2 > CLOUDY_MIN_IRR_WM2)
      cloudIrrBaseline = cloudIrrBaseline * (1.0f - CLOUDY_BASELINE_ALPHA)
                         + solarIrradianceWm2 * CLOUDY_BASELINE_ALPHA;
  }

  // Trigger: significant relative drop from established baseline on either sensor.
  const bool pvLow  = (cloudPvBaseline  > CLOUDY_MIN_PV_W    &&
                        pvPower          < cloudPvBaseline  * (1.0f - CLOUDY_PV_DROP_FRAC));
  const bool irrLow = (tslSensorOk &&
                        cloudIrrBaseline > CLOUDY_MIN_IRR_WM2 &&
                        solarIrradianceWm2 < cloudIrrBaseline * (1.0f - CLOUDY_IRR_DROP_FRAC));
  const bool lowLight = pvLow || irrLow;

  if (!cloudyCondition) {
    if (lowLight) {
      if (cloudyStartMs == 0) cloudyStartMs = now;
      if (now - cloudyStartMs >= CLOUDY_CONFIRM_MS) {
        cloudyCondition = true;
        cloudyStartMs   = 0;
        clearStartMs    = 0;
        Serial.printf("CLOUD: confirmed — pv=%.1fW (ref=%.1fW)  irr=%.0f W/m² (ref=%.0f W/m²)\n",
                      pvPower, cloudPvBaseline, solarIrradianceWm2, cloudIrrBaseline);
      }
    } else {
      cloudyStartMs = 0;
    }
  } else {
    if (!lowLight) {
      if (clearStartMs == 0) clearStartMs = now;
      if (now - clearStartMs >= CLOUDY_CLEAR_MS) {
        cloudyCondition = false;
        clearStartMs    = 0;
        cloudyStartMs   = 0;
        Serial.printf("CLOUD: cleared  — pv=%.1fW (ref=%.1fW)  irr=%.0f W/m² (ref=%.0f W/m²)\n",
                      pvPower, cloudPvBaseline, solarIrradianceWm2, cloudIrrBaseline);
      }
    } else {
      clearStartMs = 0;
    }
  }
}

void serviceSensors() {
  static unsigned long lastSensorSample = 0;
  static unsigned long lastAuxSample    = 0;
  unsigned long now = millis();
  if (now - lastSensorSample >= SENSOR_SAMPLE_INTERVAL_MS) {
    lastSensorSample = now;
    sampleSensors();
    updateEnergy();
  }
  // Safety: invalid sensor data must not continue driving CC/CV/MPPT.
  // If either INA260 is not OK while charging, stop immediately.
  if (mpptRunning && (!pvSensorOk || !battSensorOk)) {
    stopCharging(!pvSensorOk
      ? "Sensor fault: PV INA260 unavailable -- charging halted for safety"
      : "Sensor fault: Battery INA260 unavailable -- charging halted for safety",
      STOP_FAULT);
  }

  // Thermal protection: stop charging if battery NTC exceeds limit.
  if (mpptRunning && ntcValid && batteryTempC > MAX_BATTERY_TEMP_C) {
    stopCharging("Thermal protection: battery temperature " +
                 String(batteryTempC, 1) + "°C exceeded " +
                 String(MAX_BATTERY_TEMP_C, 0) + "°C limit",
                 STOP_FAULT);
  }

  if (now - lastAuxSample >= AUX_SAMPLE_INTERVAL_MS) {
    lastAuxSample = now;
    sampleAuxSensors();
    updateCloudyDetector();
  }
}

// Deliberately does NOT call mcpwm_gpio_init() here -- GPIO18/19 must stay
// disconnected from the MCPWM peripheral until gatesResume() attaches them
// after a validated start. Premature attachment would let the configured
// waveform reach the gate driver before gatesForceOff() runs.
void pwmInit(float initialDutyPercent) {
  mcpwm_config_t pwmConfig = {};
  pwmConfig.frequency    = PWM_FREQUENCY_HZ;
  pwmConfig.cmpr_a       = initialDutyPercent;
  pwmConfig.cmpr_b       = 0.0f;
  pwmConfig.duty_mode    = MCPWM_DUTY_MODE_0;
  pwmConfig.counter_mode = MCPWM_UP_COUNTER;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig);

  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        PWM_DEADTIME_TICKS, PWM_DEADTIME_TICKS);
}

void pwmSetDuty(float percent) {
  if (percent < PWM_DUTY_HARD_MIN_PERCENT) percent = PWM_DUTY_HARD_MIN_PERCENT;
  if (percent > PWM_DUTY_HARD_MAX_PERCENT) percent = PWM_DUTY_HARD_MAX_PERCENT;
  dutyPercent = percent;

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, percent);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
}

// Plain GPIO low -- safe before mcpwm_init() has run.
void forcePinsLowRaw() {
  pinMode(PWM_HIGH_SIDE_PIN, OUTPUT);
  digitalWrite(PWM_HIGH_SIDE_PIN, LOW);
  pinMode(PWM_LOW_SIDE_PIN, OUTPUT);
  digitalWrite(PWM_LOW_SIDE_PIN, LOW);
}

// Stop MCPWM and force both IR2101 gate signals low.
// Calling mcpwm_stop alone leaves the last duty on the output; forcePinsLowRaw()
// guarantees HIN=LOW and LIN=LOW so the half-bridge sits in a safe idle state.
void gatesForceOff() {
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  forcePinsLowRaw();
}

// Re-bind GPIO pins to the MCPWM peripheral and restart the timer.
// Must be called after gatesForceOff() before any new duty can be applied.
void gatesResume() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_HIGH_SIDE_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_LOW_SIDE_PIN);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

// Stop the charger, record the reason and stop type, and cut the gates.
// STOP_AUTO_PV timestamps the stop so the auto-restart timer can run.
void stopCharging(const String &reason, StopType stopType) {
  gatesForceOff();
  mpptRunning    = false;
  chargeStage    = STAGE_STOPPED;
  lastStopReason = reason;
  lastStopType   = stopType;
  hasSeenCurrentFlow = false;
  if (stopType == STOP_AUTO_PV) lastAutoStopTime = millis();
}

// Feedforward starting-duty estimate (Fractional Open-Circuit Voltage).
// Gates are off until Start is pressed so pvVoltage is a true Voc measurement.
float computeFeedforwardDutyPercent() {
  float vocNow      = pvVoltage;
  float vmpEstimate = PANEL_FOCV_RATIO * vocNow;
  float duty        = (batteryVoltage / vmpEstimate) * 100.0f;

  duty = constrain(duty, MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT);

  Serial.printf("Feedforward duty: Voc=%.2fV Vmp_est=%.2fV Vbatt=%.2fV -> duty=%.2f%%\n",
                vocNow, vmpEstimate, batteryVoltage, duty);
  lastFeedforwardDutyPercent = duty;
  lastVmpEstimate            = vmpEstimate;
  return duty;
}

// Validate preconditions, compute the mode-specific starting duty, then
// arm the gates and transition to STAGE_RAMPING (MPPT/HYBRID) or STAGE_BULK (PWM_AUTO).
//
// Starting duty selection:
//   MPPT     — FOCV feedforward only; AI has no involvement.
//   HYBRID   — AI predicted duty when model is warm; falls back to FOCV.
//   PWM_AUTO — fixed MPPT_DUTY_MIN_PERCENT; CC controller ramps up naturally.
//
// Returns true on success; on failure fills errorOut and returns false without
// touching the gates.
bool startCharging(String &errorOut) {
  if (!pvSensorOk || !battSensorOk) {
    errorOut = String(!pvSensorOk ? "PV" : "Battery") + " INA260 sensor not available";
    return false;
  }

  float minRequired = batteryFullVoltage() + MPPT_MIN_PV_HEADROOM_START_V;
  if (pvVoltage < minRequired) {
    errorOut = "PV voltage " + String(pvVoltage, 2) + "V is below minimum required (" +
               String(batteryFullVoltage(), 2) + "V target + " + String(MPPT_MIN_PV_HEADROOM_START_V, 1) +
               "V start headroom = " + String(minRequired, 2) + "V)";
    return false;
  }

  if (operatingMode == MODE_MPPT) {
    // Pure P&O — FOCV feedforward only. AI has no involvement in MPPT mode.
    dutyPercent = computeFeedforwardDutyPercent();
  } else if (operatingMode == MODE_HYBRID) {
    float ff = computeFeedforwardDutyPercent();
    // Level 1 (Hybrid only): AI replaces FOCV starting duty when model is warm.
    if (regValid && regPredDutyPct >= MPPT_DUTY_MIN_PERCENT) {
      Serial.printf("AI start (Hybrid L1): predicted duty %.2f%% (FOCV was %.2f%%)\n", regPredDutyPct, ff);
      dutyPercent = regPredDutyPct;
    } else {
      dutyPercent = ff;
    }
  } else if (operatingMode == MODE_PWM_AUTO) {
    // CC-CV auto: start low and ramp up under current control — no ramp stage needed.
    dutyPercent = MPPT_DUTY_MIN_PERCENT;
  }
  previousPvPower  = 0.0f;
  perturbDirection = 1;

  if (operatingMode == MODE_PWM_AUTO) {
    // Skip soft-start ramp — proportional CC controller handles inrush naturally.
    rampCeilingVoltage = batteryFullVoltage();
    rampStepPerCycle   = 0.0f;
    chargeStage        = STAGE_BULK;
  } else {
    float fullV = batteryFullVoltage();
    rampCeilingVoltage = batteryVoltage;
    int totalSteps   = RAMP_DURATION_MS / MPPT_STEP_INTERVAL_MS;
    rampStepPerCycle = (fullV - rampCeilingVoltage) / (float)totalSteps;
    if (rampStepPerCycle < 0) rampStepPerCycle = 0;
    chargeStage = STAGE_RAMPING;
  }

  pwmSetDuty(dutyPercent);
  gatesResume();
  mpptRunning        = true;
  lastMpptStep       = millis();
  chargingStartTime  = millis();
  hasSeenCurrentFlow = false;
  resetEnergySession();
  return true;
}

// ---- Shared charge-controller helpers ----------------------------------------

// Advance the ramp ceiling one step toward full-charge voltage and update chargeStage.
static void advanceRampAndStage() {
  float fullV = batteryFullVoltage();
  if (rampCeilingVoltage < fullV) {
    rampCeilingVoltage = min(rampCeilingVoltage + rampStepPerCycle, fullV);
  }
  if (batteryVoltage >= fullV - BATTERY_FULL_TOLERANCE_V)
    chargeStage = STAGE_HOLDING;
  else if (rampCeilingVoltage >= fullV)
    chargeStage = STAGE_BULK;
}

// Apply CC (current cap) and CV (voltage ceiling) limits to duty, then clamp to MPPT range.
static void applyCC_CV(float &duty) {
  if (batteryCurrent > MAX_CHARGE_CURRENT_A || batteryVoltage > rampCeilingVoltage)
    duty -= CHARGE_CONTROL_CORRECTION_STEP_PERCENT;
  duty = constrain(duty, MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT);
}

// Forward declarations for PWM_AUTO helpers used by mpptStep and hybridStep.
float pwmAbsorbVoltage();
float pwmFloatVoltage();

// ---- Shared CC-CV stage helpers (used by all three step functions) ----------

// Float stage: proportional CV at float voltage; re-bulks on voltage droop.
static void handleFloatStage(float &duty, const char *label) {
  const float floatV = pwmFloatVoltage();
  if (batteryVoltage < floatV - PWM_REBULK_HYST_V) {
    chargeStage = STAGE_BULK;
    Serial.printf("%s: voltage droop — re-entering Bulk\n", label);
  } else {
    const float corr = constrain(PWMCV_KP_VOLTAGE * (floatV - batteryVoltage),
                                 -PWMCC_MAX_STEP, PWMCC_MAX_STEP);
    duty = constrain(duty + corr, MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT);
  }
}

// Absorption stage: proportional CV at temperature-compensated target.
// Returns false (and has called stopCharging) when the charge cycle ends — caller must return.
static bool handleAbsorptionStage(float &duty, const char *label) {
  const float absorbV = pwmAbsorbVoltage();
  const float corr    = constrain(PWMCV_KP_VOLTAGE * (absorbV - batteryVoltage),
                                  -PWMCC_MAX_STEP, PWMCC_MAX_STEP);
  duty = constrain(duty + corr, MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT);
  if (hasSeenCurrentFlow &&
      batteryVoltage >= absorbV - BATTERY_FULL_TOLERANCE_V &&
      batteryCurrent <= TERMINATION_CURRENT_A &&
      millis() - chargingStartTime >= MIN_CHARGE_RUNTIME_BEFORE_TERMINATION_MS) {
    if (pwmFloatVoltage() > 0.0f) {
      chargeStage = STAGE_FLOAT;
      Serial.printf("%s: absorption complete — entering Float\n", label);
    } else {
      stopCharging("Charging complete — battery full (" +
                   String(CHEMISTRY_NAMES[selectedChemistry]) + ")", STOP_COMPLETE);
      return false;
    }
  }
  return true;
}

// Apply duty to hardware and update bookkeeping counters.
static void commitDuty(float duty) {
  pwmSetDuty(duty);
  perturbCount++;
  if (batteryCurrent > CURRENT_FLOW_DETECT_A) hasSeenCurrentFlow = true;
}

// ---- P&O and fixed-PWM controllers ------------------------------------------

// P&O MPPT with full CC-CV stage machine.
//
// Bulk/Ramping: fixed MPPT_DUTY_STEP_PERCENT step each tick; direction reverses
// when pvPower < previousPvPower.  CC and ramp-ceiling hard limits on top.
//
// Absorption (STAGE_HOLDING): P&O hands over to proportional CV control at the
// temperature-compensated absorption voltage.  Terminates on current taper or
// transitions to Float for LiFePO4.
//
// Float (STAGE_FLOAT, LiFePO4 only): proportional CV at float voltage.
// Re-enters Bulk on voltage droop > PWM_REBULK_HYST_V.
void mpptStep() {
  float duty = dutyPercent;

  if (chargeStage == STAGE_FLOAT) {
    handleFloatStage(duty, "MPPT");
  } else if (chargeStage == STAGE_HOLDING) {
    if (!handleAbsorptionStage(duty, "MPPT")) return;
  } else {
    // Bulk / Ramping: fixed-step P&O, CC + ramp-ceiling hard limits on top.
    if (pvPower < previousPvPower) perturbDirection = -perturbDirection;
    duty = constrain(duty + perturbDirection * MPPT_DUTY_STEP_PERCENT,
                     MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT);
    previousPvPower = pvPower;
    advanceRampAndStage();
    applyCC_CV(duty);
  }

  commitDuty(duty);
}

// HYB-AIMLSL controller: AI-guided P&O in Bulk, proportional CV in Absorption/Float.
//
// Bulk / Ramping — three AI zones:
//   Hold (gap ≤ HOLD_BAND + stable power): skip perturbation. Band doubles when cloudy.
//   Far  (gap > NEAR_BAND): AI overrides direction, large step. Disabled → P&O + std step.
//   Near (HOLD_BAND < gap ≤ NEAR_BAND): P&O direction, fine step. Disabled → std step.
//   Falls back to standard P&O when regValid is false.
//
// Absorption (STAGE_HOLDING): AI hands over to proportional CV at temperature-compensated
// target voltage. Terminates on current taper or transitions to Float for LiFePO4.
//
// Float (STAGE_FLOAT, LiFePO4 only): proportional CV at float voltage; re-bulks on droop.
void hybridStep() {
  float duty = dutyPercent;

  if (chargeStage == STAGE_FLOAT) {
    handleFloatStage(duty, "HYBRID");
  } else if (chargeStage == STAGE_HOLDING) {
    if (!handleAbsorptionStage(duty, "HYBRID")) return;
  } else {
    // Bulk / Ramping: AI-guided P&O with three zones.
    float step = MPPT_DUTY_STEP_PERCENT;

    if (regValid) {
      const float diff    = regPredDutyPct - dutyPercent;
      const float absDiff = fabsf(diff);

      const float holdBand = cloudyCondition ? HYBRID_HOLD_BAND_PERCENT * 2.0f
                                             : HYBRID_HOLD_BAND_PERCENT;
      if (hybZoneHoldEnabled &&
          absDiff <= holdBand &&
          fabsf(pvPower - previousPvPower) < HYBRID_HOLD_POWER_W) {
        previousPvPower = pvPower;
        advanceRampAndStage();
        applyCC_CV(duty);
        commitDuty(duty);
        return;
      }

      if (absDiff > HYBRID_NEAR_BAND_PERCENT) {
        if (hybZoneFarEnabled) {
          perturbDirection = (diff > 0.0f) ? 1 : -1;
          step = HYBRID_STEP_FAR_PERCENT;
        } else {
          if (pvPower < previousPvPower) perturbDirection = -perturbDirection;
        }
      } else {
        if (pvPower < previousPvPower) perturbDirection = -perturbDirection;
        if (hybZoneNearEnabled) step = HYBRID_STEP_NEAR_PERCENT;
      }
    } else {
      if (pvPower < previousPvPower) perturbDirection = -perturbDirection;
    }

    duty = constrain(duty + perturbDirection * step,
                     MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT);
    previousPvPower = pvPower;
    advanceRampAndStage();
    applyCC_CV(duty);
  }

  commitDuty(duty);
}

// ---- Auto PWM (commercial CC-CV) helpers ------------------------------------

// Temperature-compensated absorption voltage.
// Reduces absorption target by 3mV/°C/cell above 25°C to prevent overcharging hot packs.
float pwmAbsorbVoltage() {
  float base = batteryFullVoltage();
  if (ntcValid && batteryTempC > NTC_T_NOMINAL_C) {
    float comp = (batteryTempC - NTC_T_NOMINAL_C) * (PWM_TEMP_COMP_MV_CELL / 1000.0f) * selectedCellCount;
    base -= comp;
  }
  return base;
}

// Float target: only used for LiFePO4. Returns 0 for other chemistries (no float stage).
float pwmFloatVoltage() {
  if (selectedChemistry != CHEM_LIFEPO4) return 0.0f;
  return PWM_LIFEPO4_FLOAT_V_CELL * selectedCellCount;
}

// Commercial CC-CV controller:
//   BULK    (STAGE_BULK)   : proportional current control to MAX_CHARGE_CURRENT_A.
//   HOLDING (STAGE_HOLDING): proportional voltage control to absorption target; exit at C/20.
//   FLOAT   (STAGE_FLOAT)  : proportional voltage control to float target (LiFePO4 only);
//                            re-bulk if voltage droops below float − PWM_REBULK_HYST_V.
void pwmAutoStep() {
  float duty = dutyPercent;

  if (chargeStage == STAGE_FLOAT) {
    handleFloatStage(duty, "PWM_AUTO");
  } else if (chargeStage == STAGE_HOLDING) {
    if (!handleAbsorptionStage(duty, "PWM_AUTO")) return;
  } else {
    // BULK (CC): proportional current control toward MAX_CHARGE_CURRENT_A.
    const float corr = constrain(PWMCC_KP_CURRENT * (MAX_CHARGE_CURRENT_A - batteryCurrent),
                                 -PWMCC_MAX_STEP, PWMCC_MAX_STEP);
    duty = constrain(duty + corr, MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT);
    if (batteryVoltage >= pwmAbsorbVoltage() - BATTERY_FULL_TOLERANCE_V) {
      chargeStage = STAGE_HOLDING;
      Serial.println("PWM_AUTO: target voltage reached — entering Absorption (CV)");
    }
    applyCC_CV(duty);  // hard current cap backstop
  }

  commitDuty(duty);
}

// Vmp predictor — observation only, never touches PWM or P&O variables.
// Runs every MPPT_STEP_INTERVAL_MS. Output is regPredVmpV (V) and regPredDutyPct (%).
//
// Feature vector (14): instantaneous panel/battery electrical state + delta values
// computed from the previous tick.  Order must match vmp_scaler.json exactly.
//
// Transient recovery: if |ΔPpv| > 2W the prediction is too noisy to trust.
// vmpWasTransient is set true and regValid=false for that tick.  On the next valid
// tick serviceCharger() uses vmpTransientRecovery to re-seed the HYBRID duty from
// the fresh prediction, preventing P&O from diverging after a cloud shadow clears.
void runVmpInference() {
  static float regPrevPvVoltage = 0.0f;
  static float regPrevPvCurrent = 0.0f;
  static float regPrevPvPower   = 0.0f;
  static bool  vmpWasTransient  = false;
  if (!pvSensorOk || !battSensorOk) { regValid = false; return; }

  const float dPvPower = pvPower - regPrevPvPower;

  // Feature order must match vmp_scaler.json exactly (14 features):
  const float feat[VMP_N_FEATURES] = {
    pvVoltage,                                                       // pv_voltage_V
    pvCurrent,                                                       // pv_current_A
    pvPower,                                                         // pv_power_W
    batteryVoltage,                                                  // battery_voltage_V
    batteryCurrent,                                                  // battery_current_A
    luxValue,                                                        // lux
    ambientTempC,                                                    // panel_temperature_C
    dutyPercent,                                                     // duty_cycle_pct
    pvVoltage - regPrevPvVoltage,                                    // delta_pv_voltage_V
    dPvPower,                                                        // delta_pv_power_W
    fabsf(dPvPower),                                                 // abs_delta_pv_power
    dPvPower > 0.0f ? 1.0f : (dPvPower < 0.0f ? -1.0f : 0.0f),    // power_change_sign
    pvCurrent - regPrevPvCurrent,                                    // delta_pv_current_A
    (float)chargeStage                                               // mppt_state_code
  };

  regPrevPvVoltage = pvVoltage;
  regPrevPvCurrent = pvCurrent;
  regPrevPvPower   = pvPower;

  // Skip on transient spikes — mark for Level 2 recovery on the next valid step.
  if (fabsf(dPvPower) > 2.0f) {
    vmpWasTransient = true;
    regValid = false;
    return;
  }

  float out[1];
  vmpForwardPass(feat, out);   // normalises internally; out[0] = predicted Vmp (V)

  const float vmp = out[0];

  // Reject predictions outside the model's training range (10–20 V).
  if (vmp < 10.0f || vmp > 20.0f) { regValid = false; return; }

  regPredVmpV = vmp;

  // Feedforward duty: duty = Vbat / (0.95 × Vmp) × 100
  // 0.95 = assumed buck efficiency — accounts for ~5% conversion loss.
  // OBSERVATION ONLY — never written to PWM directly; used only as a P&O seed.
  regPredDutyPct = constrain((batteryVoltage / (0.95f * vmp)) * 100.0f,
                             MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT);

  // Signal transient recovery for serviceCharger() to act on this tick.
  if (vmpWasTransient) {
    vmpTransientRecovery = true;
    vmpWasTransient = false;
  }

  regValid = true;
}

void checkAutoRestart() {
  if (!autoRestartEnabled)              return;
  if (mpptRunning)                      return;
  if (lastStopType != STOP_AUTO_PV)     return;
  if (millis() - lastAutoStopTime < AUTO_RESTART_INTERVAL_MS) return;

  lastAutoStopTime = millis();  // reset timer before attempt
  String err;
  if (startCharging(err)) {
    lastStopReason = "Auto-restarted: PV recovered to " + String(pvVoltage, 2) + "V";
    Serial.println("Auto-restart: PV=" + String(pvVoltage, 2) + "V");
  } else {
    Serial.println("Auto-restart check: not ready -- " + err);
  }
}

// Main charger tick — called every loop().
// Runs inference, handles the AI transient re-seed (HYBRID only), then dispatches
// to the active step function (mpptStep / hybridStep / pwmAutoStep) on each
// MPPT_STEP_INTERVAL_MS boundary.  Also handles auto-stop on PV undervoltage and
// auto-restart polling when the charger is idle.
void serviceCharger() {
  // Auto-stop if PV no longer has enough headroom over battery target.
  if (mpptRunning && pvVoltage < (batteryFullVoltage() + MPPT_MIN_PV_HEADROOM_RUNNING_V)) {
    stopCharging("Auto-stopped: PV voltage " + String(pvVoltage, 2) + "V dropped below " +
                 String(batteryFullVoltage(), 2) + "V target + " +
                 String(MPPT_MIN_PV_HEADROOM_RUNNING_V, 1) + "V running headroom",
                 STOP_AUTO_PV);
    return;
  }
  if (!mpptRunning) { checkAutoRestart(); return; }

  unsigned long now = millis();
  if (now - lastMpptStep >= MPPT_STEP_INTERVAL_MS) {
    lastMpptStep = now;
    runVmpInference();

    // Level 2 (Hybrid only): re-seed from AI predicted duty after a power transient clears.
    // MPPT is pure P&O — no AI re-seeding there.
    if (vmpTransientRecovery && operatingMode == MODE_HYBRID && regValid) {
      Serial.printf("AI re-seed after transient: %.2f%% -> %.2f%%\n",
                    dutyPercent, regPredDutyPct);
      dutyPercent     = regPredDutyPct;
      previousPvPower = 0.0f;
      vmpTransientRecovery = false;
    }
    vmpTransientRecovery = false;

    if      (operatingMode == MODE_MPPT)     mpptStep();
    else if (operatingMode == MODE_HYBRID)   hybridStep();
    else if (operatingMode == MODE_PWM_AUTO) pwmAutoStep();
  }
}

// Escape a String for safe embedding in a JSON string literal.
// Replaces \ → \\ and " → \" in-place and returns a C-string pointer.
static const char *jsonEscape(const String &s, char *out, size_t outLen) {
  size_t j = 0;
  for (size_t i = 0; i < s.length() && j + 2 < outLen; i++) {
    char c = s[i];
    if (c == '"' || c == '\\') out[j++] = '\\';
    out[j++] = c;
  }
  out[j] = '\0';
  return out;
}

// Serialize the complete runtime snapshot as a JSON object and send it to the dashboard.
// All live measurements, stage/mode state, sensor health flags, energy totals,
// AI prediction outputs, cloudy flag, and zone enables are included.
void handleState() {
  static char buf[2048];
  static char escapedMsg[256];
  float fullV = batteryFullVoltage();
  jsonEscape(lastStopReason, escapedMsg, sizeof(escapedMsg));
  snprintf(buf, sizeof(buf),
    "{"
    "\"pvVoltage\":%.3f,"
    "\"pvCurrent\":%.3f,"
    "\"pvPower\":%.3f,"
    "\"batteryVoltage\":%.3f,"
    "\"batteryCurrent\":%.3f,"
    "\"batteryPower\":%.3f,"
    "\"dutyPercent\":%.2f,"
    "\"dutyMin\":%.1f,"
    "\"dutyMax\":%.1f,"
    "\"perturbDirection\":%d,"
    "\"mpptRunning\":%s,"
    "\"minStartVoltage\":%.2f,"
    "\"feedforwardDutyPercent\":%.2f,"
    "\"referenceVmpV\":%.3f,"
    "\"statusMessage\":\"%s\","
    "\"chemistry\":%d,"
    "\"chemistryName\":\"%s\","
    "\"cellCount\":%d,"
    "\"batteryFullVoltage\":%.2f,"
    "\"batteryEmptyVoltage\":%.2f,"
    "\"rampCeilingVoltage\":%.2f,"
    "\"maxChargeCurrent\":%.2f,"
    "\"terminationCurrent\":%.2f,"
    "\"chargeStage\":\"%s\","
    "\"rawPvVoltage\":%.3f,"
    "\"rawBattVoltage\":%.3f,"
    "\"rawPvCurrent\":%.3f,"
    "\"rawBattCurrent\":%.3f,"
    "\"pvSensorOk\":%s,"
    "\"battSensorOk\":%s,"
    "\"tmpSensorOk\":%s,"
    "\"tslSensorOk\":%s,"
    "\"temperature\":%.2f,"
    "\"lux\":%.1f,"
    "\"irradiance\":%.2f,"
    "\"batteryTempC\":%.2f,"
    "\"ntcValid\":%s,"
    "\"perturbCount\":%u,"
    "\"pvEnergyWh\":%.4f,"
    "\"batteryEnergyWh\":%.4f,"
    "\"instantEfficiency\":%.2f,"
    "\"cumulativeEfficiency\":%.2f,"
    "\"efficiencyValid\":%s,"
    "\"effSuspicious\":%s,"
    "\"sessionId\":%u,"
    "\"operatingMode\":\"%s\","
    "\"autoRestartEnabled\":%s,"
    "\"autoRestartEligible\":%s,"
    "\"autoRestartCountdown\":%u,"
    "\"regValid\":%s,"
    "\"regPredVmpV\":%.3f,"
    "\"regPredDutyPct\":%.2f,"
    "\"cloudyCondition\":%s,"
    "\"hybZoneFarEnabled\":%s,"
    "\"hybZoneNearEnabled\":%s,"
    "\"hybZoneHoldEnabled\":%s"
    "}",
    pvVoltage, pvCurrent, pvPower,
    batteryVoltage, batteryCurrent, batteryPower,
    dutyPercent, MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT,
    (int)perturbDirection,
    mpptRunning ? "true" : "false",
    fullV + MPPT_MIN_PV_HEADROOM_START_V,
    lastFeedforwardDutyPercent,
    lastVmpEstimate,
    escapedMsg,
    (int)selectedChemistry,
    CHEMISTRY_NAMES[selectedChemistry],
    selectedCellCount,
    fullV,
    batteryEmptyVoltage(),
    rampCeilingVoltage,
    MAX_CHARGE_CURRENT_A,
    TERMINATION_CURRENT_A,
    chargeStageStr(),
    rawPvVoltage, rawBattVoltage, rawPvCurrent, rawBattCurrent,
    pvSensorOk   ? "true" : "false",
    battSensorOk ? "true" : "false",
    tmpSensorOk  ? "true" : "false",
    tslSensorOk  ? "true" : "false",
    ambientTempC,
    luxValue,
    solarIrradianceWm2,
    batteryTempC,
    ntcValid ? "true" : "false",
    perturbCount,
    pvEnergyWh, batteryEnergyWh,
    efficiencyValid ? instantEfficiency    : 0.0f,
    pvEnergyWh > 0.001f ? cumulativeEfficiency : 0.0f,
    efficiencyValid  ? "true" : "false",
    effSuspicious    ? "true" : "false",
    sessionId,
    operatingMode == MODE_MPPT ? "MPPT" :
      operatingMode == MODE_HYBRID   ? "HYBRID" :
      operatingMode == MODE_PWM_AUTO ? "PWM_AUTO" : "MPPT",
    autoRestartEnabled ? "true" : "false",
    (lastStopType == STOP_AUTO_PV) ? "true" : "false",
    (!mpptRunning && autoRestartEnabled && lastStopType == STOP_AUTO_PV)
      ? (millis() - lastAutoStopTime >= AUTO_RESTART_INTERVAL_MS
          ? 0u
          : (uint32_t)((AUTO_RESTART_INTERVAL_MS - (millis() - lastAutoStopTime)) / 1000))
      : 0u,
    regValid         ? "true" : "false",
    regPredVmpV,
    regPredDutyPct,
    cloudyCondition  ? "true" : "false",
    hybZoneFarEnabled  ? "true" : "false",
    hybZoneNearEnabled ? "true" : "false",
    hybZoneHoldEnabled ? "true" : "false"
  );
  server.send(200, "application/json", buf);
}

void handleStart() {
  String error;
  if (startCharging(error)) {
    lastStopReason = "Charging running";
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(200, "application/json", "{\"ok\":false,\"reason\":\"" + error + "\"}");
  }
}

void handleStop() {
  stopCharging("Stopped by user");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSetChemistry() {
  if (mpptRunning) {
    server.send(200, "application/json", "{\"ok\":false,\"reason\":\"Stop charging before changing chemistry\"}");
    return;
  }
  if (!server.hasArg("chem")) {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Missing chem parameter\"}");
    return;
  }
  int chem = server.arg("chem").toInt();
  if (chem < 0 || chem > 2) {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Invalid chemistry\"}");
    return;
  }
  selectedChemistry = (BatteryChemistry)chem;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSetCellCount() {
  if (mpptRunning) {
    server.send(200, "application/json", "{\"ok\":false,\"reason\":\"Stop charging before changing cell count\"}");
    return;
  }
  if (!server.hasArg("cells")) {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Missing cells parameter\"}");
    return;
  }
  int cells = server.arg("cells").toInt();
  if (cells < BATTERY_CELL_COUNT_MIN || cells > BATTERY_CELL_COUNT_MAX) {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Invalid cell count\"}");
    return;
  }
  selectedCellCount = cells;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleResetEnergy() {
  resetEnergySession();
  Serial.println("Energy session reset by dashboard");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSetAutoRestart() {
  if (!server.hasArg("enable")) {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Missing enable parameter\"}");
    return;
  }
  autoRestartEnabled = server.arg("enable") == "1";
  server.send(200, "application/json", autoRestartEnabled
    ? "{\"ok\":true,\"autoRestart\":true}"
    : "{\"ok\":true,\"autoRestart\":false}");
}

void handleSetMode() {
  if (mpptRunning) {
    server.send(200, "application/json", "{\"ok\":false,\"reason\":\"Stop charging before changing mode\"}");
    return;
  }
  if (!server.hasArg("mode")) {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Missing mode parameter\"}");
    return;
  }
  String m = server.arg("mode");
  if (m == "MPPT") {
    operatingMode = MODE_MPPT;
    server.send(200, "application/json", "{\"ok\":true,\"mode\":\"MPPT\"}");
  } else if (m == "HYBRID") {
    operatingMode = MODE_HYBRID;
    server.send(200, "application/json", "{\"ok\":true,\"mode\":\"HYBRID\"}");
  } else if (m == "PWM_AUTO") {
    operatingMode = MODE_PWM_AUTO;
    server.send(200, "application/json", "{\"ok\":true,\"mode\":\"PWM_AUTO\"}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Invalid mode (use MPPT, HYBRID, or PWM_AUTO)\"}");
  }
}

void handleSetHybZones() {
  if (server.hasArg("far"))  hybZoneFarEnabled  = (server.arg("far")  == "1");
  if (server.hasArg("near")) hybZoneNearEnabled = (server.arg("near") == "1");
  if (server.hasArg("hold")) hybZoneHoldEnabled = (server.arg("hold") == "1");
  Serial.printf("HYB zones: far=%d near=%d hold=%d\n",
                hybZoneFarEnabled, hybZoneNearEnabled, hybZoneHoldEnabled);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void setup() {
  forcePinsLowRaw();  // drive HIN/LIN low before anything else

  Serial.begin(115200);
  delay(100);  // let serial settle before first print

  // ---- NTC ADC pin ----
  // ESP32 default attenuation is ADC_0db (0–1.1V). The NTC divider midpoint
  // is 1.65V at 25°C -- above 1.1V. ADC_11db extends the range to 0–3.3V.
  analogSetPinAttenuation(NTC_PIN, ADC_11db);
  Serial.printf("NTC battery sensor on GPIO%d (ADC1, 11dB attenuation)\n", NTC_PIN);

  // ---- I2C init ----
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);  // 100kHz -- more robust than 400kHz with switching noise on the bus
  Wire.setTimeOut(50);    // 50ms I2C timeout -- prevents infinite stall if sensor doesn't ACK

  // ---- INA260 init ----
  Serial.println("Initialising INA260 sensors...");
  pvSensorOk   = ina260Pv.begin(INA260_PV_ADDRESS);
  battSensorOk = ina260Batt.begin(INA260_BATTERY_ADDRESS);

  if (pvSensorOk) {
    // 16-sample hardware averaging: 16 × 1.1ms = ~17.6ms window.
    // Filters out most 20kHz switching noise without adding significant control-loop delay.
    ina260Pv.setAveragingCount(INA260_COUNT_16);
    ina260Pv.setCurrentConversionTime(INA260_TIME_1_1_ms);
    ina260Pv.setVoltageConversionTime(INA260_TIME_1_1_ms);
    Serial.printf("PV INA260 (0x%02X): OK\n", INA260_PV_ADDRESS);
  } else {
    Serial.printf("WARNING: PV INA260 (0x%02X) not found -- PV measurements will be zero\n",
                  INA260_PV_ADDRESS);
  }

  if (battSensorOk) {
    ina260Batt.setAveragingCount(INA260_COUNT_16);
    ina260Batt.setCurrentConversionTime(INA260_TIME_1_1_ms);
    ina260Batt.setVoltageConversionTime(INA260_TIME_1_1_ms);
    Serial.printf("Battery INA260 (0x%02X): OK\n", INA260_BATTERY_ADDRESS);
  } else {
    Serial.printf("WARNING: Battery INA260 (0x%02X) not found -- battery measurements will be zero\n",
                  INA260_BATTERY_ADDRESS);
  }

  // ---- TMP117 temperature sensor ----
  tmpSensorOk = tmp117.begin(TMP117_ADDRESS);
  if (tmpSensorOk) {
    Serial.printf("TMP117 (0x%02X): OK\n", TMP117_ADDRESS);
  } else {
    Serial.printf("WARNING: TMP117 (0x%02X) not found -- temperature will read 0\n", TMP117_ADDRESS);
  }

  // ---- TSL2591 light sensor ----
  tslSensorOk = tsl2591.begin();
  if (tslSensorOk) {
    tsl2591.setGain(TSL2591_GAIN_LOW);
    tsl2591.setTiming(TSL2591_INTEGRATIONTIME_100MS);
    Serial.printf("TSL2591 (0x%02X): OK\n", TSL2591_ADDR);
  } else {
    Serial.printf("WARNING: TSL2591 (0x%02X) not found -- lux will read 0\n", TSL2591_ADDR);
  }

  // ---- Filesystem ----
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  // ---- WiFi AP ----
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);  // disable modem sleep — prevents AP from dropping clients
  WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 6);  // channel 6: stable, low interference
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP started. Connect to Wi-Fi \"");
  Serial.print(AP_SSID);
  Serial.print("\" and open http://");
  Serial.println(apIP);

  // ---- HTTP routes ----
  // GET  /          — serve dashboard (index.html from LittleFS)
  // GET  /state     — JSON snapshot of all runtime state (polled ~1 Hz by dashboard)
  // POST /start     — arm gates and begin charging
  // POST /stop      — stop charging (user-initiated)
  // POST /chemistry — set battery chemistry (chem=0|1|2); rejects if running
  // POST /cellcount — set cell count (cells=2|3); rejects if running
  // POST /resetenergy   — zero Wh accumulators and restart session
  // POST /autorestart   — enable/disable auto-restart on PV recovery (enable=0|1)
  // POST /mode          — select operating mode (mode=MPPT|HYBRID|PWM_AUTO); rejects if running
  // POST /hybzones      — enable/disable HYB-AIMLSL zones (far=0|1, near=0|1, hold=0|1)
  server.serveStatic("/", LittleFS, "/index.html");
  server.on("/state",     HTTP_GET,  handleState);
  server.on("/start",     HTTP_POST, handleStart);
  server.on("/stop",      HTTP_POST, handleStop);
  server.on("/chemistry", HTTP_POST, handleSetChemistry);
  server.on("/cellcount",    HTTP_POST, handleSetCellCount);
  server.on("/resetenergy",  HTTP_POST, handleResetEnergy);
  server.on("/autorestart",  HTTP_POST, handleSetAutoRestart);
  server.on("/mode",         HTTP_POST, handleSetMode);
  server.on("/hybzones",     HTTP_POST, handleSetHybZones);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("AI: Regression model ready (observation only)");
  sampleSensors();  // seed EMA from real readings before loop starts
  pwmInit(MPPT_DUTY_START_PERCENT);
  gatesForceOff();  // boot in off state -- PWM starts only when Start is pressed
}

// Main loop — order matters:
//   1. handleClient()  — process any pending HTTP request first (low-latency dashboard response)
//   2. serviceSensors() — refresh EMA-filtered measurements and run aux sensors / cloudy detector
//   3. serviceCharger() — run one P&O / CC-CV step if the interval has elapsed
void loop() {
  server.handleClient();
  serviceSensors();
  serviceCharger();
}
