#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <driver/mcpwm.h>
#include <Adafruit_INA260.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_TSL2591.h>

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

float ambientTempC  = 0.0f;
float luxValue      = 0.0f;
float batteryTempC  = 0.0f;
bool  ntcValid      = false;

constexpr unsigned long AUX_SAMPLE_INTERVAL_MS = 2000;
unsigned long lastAuxSample = 0;

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
enum ChargeStage { STAGE_STOPPED, STAGE_RAMPING, STAGE_BULK, STAGE_HOLDING };
ChargeStage chargeStage = STAGE_STOPPED;
float rampCeilingVoltage = 0.0f;
float rampStepPerCycle   = 0.0f;

const char *chargeStageStr() {
  switch (chargeStage) {
    case STAGE_RAMPING: return "Ramping";
    case STAGE_BULK:    return "Bulk";
    case STAGE_HOLDING: return "Holding (CV)";
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
// MODE_MPPT: P&O algorithm adjusts duty each step (default).
// MODE_PWM:  Fixed duty reference; CC/CV limits still active for safety.
enum OperatingMode { MODE_MPPT, MODE_PWM };
// 0.0 = auto: compute feedforward at start (same as MPPT).
// Any value >= MPPT_DUTY_MIN_PERCENT = user-set fixed duty.
constexpr float PWM_FIXED_DUTY_AUTO = 0.0f;

WebServer server(80);

// Filtered measurement outputs consumed by CC/CV/MPPT
float pvVoltage      = 0.0f;
float pvCurrent      = 0.0f;
float pvPower        = 0.0f;   // from INA260 native power register
float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
float batteryPower   = 0.0f;   // from INA260 native power register

unsigned long lastSensorSample = 0;

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
OperatingMode operatingMode      = MODE_MPPT;
float fixedPwmDutyPercent        = PWM_FIXED_DUTY_AUTO;  // 0 = auto on first start
uint32_t perturbCount = 0;  // increments every control step; dashboard uses it to detect new perturbations
String lastStopReason = "Not started yet";
StopType lastStopType       = STOP_USER;
bool     autoRestartEnabled = false;
unsigned long lastAutoStopTime = 0;

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

// sampleAuxSensors: read TMP117 (temperature) and TSL2591 (light) at a reduced
// rate (AUX_SAMPLE_INTERVAL_MS = 2s).  These are non-critical for the control
// loop so they do NOT trigger a charge-stop on failure.
void sampleAuxSensors() {
  if (tmpSensorOk) {
    sensors_event_t tempEvent;
    tmp117.getEvent(&tempEvent);
    float t = tempEvent.temperature;
    if (!isnan(t) && t > -40.0f && t < 125.0f) {
      ambientTempC = t;
    }
  }
  if (tslSensorOk) {
    uint32_t lum  = tsl2591.getFullLuminosity();
    uint16_t ir   = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    float lux     = tsl2591.calculateLux(full, ir);
    if (!isnan(lux) && !isinf(lux) && lux >= 0.0f && lux < 200000.0f) {
      luxValue = lux;
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

void serviceSensors() {
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

void gatesForceOff() {
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  forcePinsLowRaw();
}

void gatesResume() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_HIGH_SIDE_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_LOW_SIDE_PIN);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

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

  if (duty > MPPT_DUTY_MAX_PERCENT) duty = MPPT_DUTY_MAX_PERCENT;
  if (duty < MPPT_DUTY_MIN_PERCENT) duty = MPPT_DUTY_MIN_PERCENT;

  Serial.printf("Feedforward duty: Voc=%.2fV Vmp_est=%.2fV Vbatt=%.2fV -> duty=%.2f%%\n",
                vocNow, vmpEstimate, batteryVoltage, duty);
  lastFeedforwardDutyPercent = duty;
  return duty;
}

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
    dutyPercent = computeFeedforwardDutyPercent();
  } else {
    // Fixed PWM: use user-set duty if one has been configured, otherwise
    // fall back to the same feedforward estimate MPPT uses so the starting
    // duty is always appropriate for the actual PV and battery voltages.
    float ff = computeFeedforwardDutyPercent();
    if (fixedPwmDutyPercent < MPPT_DUTY_MIN_PERCENT) {
      fixedPwmDutyPercent = ff;  // auto-compute and persist for the slider
    }
    dutyPercent = fixedPwmDutyPercent;
  }
  previousPvPower  = 0.0f;
  perturbDirection = 1;

  float fullV = batteryFullVoltage();
  rampCeilingVoltage = batteryVoltage;
  int totalSteps   = RAMP_DURATION_MS / MPPT_STEP_INTERVAL_MS;
  rampStepPerCycle = (fullV - rampCeilingVoltage) / (float)totalSteps;
  if (rampStepPerCycle < 0) rampStepPerCycle = 0;
  chargeStage = STAGE_RAMPING;

  pwmSetDuty(dutyPercent);
  gatesResume();
  mpptRunning        = true;
  lastMpptStep       = millis();
  chargingStartTime  = millis();
  hasSeenCurrentFlow = false;
  resetEnergySession();
  return true;
}

// Duty-cycle P&O MPPT: nudge duty in the direction that increased PV power last
// step; reverse when power drops. Ramp ceiling and current cap are hard limits
// layered on top -- P&O keeps running throughout (Option B).
//
// pvPower is the global updated by sampleSensors() from INA260's native power
// register. MPPT does not read sensors directly.
void mpptStep() {
  if (pvPower < previousPvPower) {
    perturbDirection = -perturbDirection;
  }

  dutyPercent += perturbDirection * MPPT_DUTY_STEP_PERCENT;
  if (dutyPercent > MPPT_DUTY_MAX_PERCENT) dutyPercent = MPPT_DUTY_MAX_PERCENT;
  if (dutyPercent < MPPT_DUTY_MIN_PERCENT) dutyPercent = MPPT_DUTY_MIN_PERCENT;
  previousPvPower = pvPower;

  // Advance the ramp ceiling toward the full-charge target.
  float fullV = batteryFullVoltage();
  if (rampCeilingVoltage < fullV) {
    rampCeilingVoltage += rampStepPerCycle;
    if (rampCeilingVoltage > fullV) rampCeilingVoltage = fullV;
  }

  // Stage reflects where the ACTUAL battery voltage is relative to target.
  if (batteryVoltage >= (fullV - BATTERY_FULL_TOLERANCE_V)) {
    chargeStage = STAGE_HOLDING;
  } else if (rampCeilingVoltage >= fullV) {
    chargeStage = STAGE_BULK;
  }

  // CC: battery current cap -- takes priority over voltage ceiling.
  // CV: voltage ceiling -- keeps battery voltage below the ramp ceiling.
  if (batteryCurrent > MAX_CHARGE_CURRENT_A) {
    dutyPercent -= CHARGE_CONTROL_CORRECTION_STEP_PERCENT;
  } else if (batteryVoltage > rampCeilingVoltage) {
    dutyPercent -= CHARGE_CONTROL_CORRECTION_STEP_PERCENT;
  }
  if (dutyPercent > MPPT_DUTY_MAX_PERCENT) dutyPercent = MPPT_DUTY_MAX_PERCENT;
  if (dutyPercent < MPPT_DUTY_MIN_PERCENT) dutyPercent = MPPT_DUTY_MIN_PERCENT;

  pwmSetDuty(dutyPercent);
  perturbCount++;

  // Track once real current has been seen this session.
  if (batteryCurrent > 0.1f) hasSeenCurrentFlow = true;

  // Termination: at target voltage, current tapered below C/20, current was
  // actually observed flowing, and minimum runtime has elapsed (prevents
  // false termination during EMA settling at startup).
  if (chargeStage == STAGE_HOLDING &&
      hasSeenCurrentFlow &&
      batteryVoltage >= (fullV - BATTERY_FULL_TOLERANCE_V) &&
      batteryCurrent <= TERMINATION_CURRENT_A &&
      (millis() - chargingStartTime) >= MIN_CHARGE_RUNTIME_BEFORE_TERMINATION_MS) {
    stopCharging("Charging complete -- battery full (" + String(CHEMISTRY_NAMES[selectedChemistry]) + ")", STOP_COMPLETE);
  }
}

// Fixed-duty controller: applies fixedPwmDutyPercent as the duty reference
// but still enforces CC/CV limits and all charge-stage logic.
// The P&O perturbation is absent — duty stays constant unless a safety limit
// forces a reduction.
void fixedPwmStep() {
  float duty = fixedPwmDutyPercent;

  float fullV = batteryFullVoltage();
  if (rampCeilingVoltage < fullV) {
    rampCeilingVoltage += rampStepPerCycle;
    if (rampCeilingVoltage > fullV) rampCeilingVoltage = fullV;
  }

  if (batteryVoltage >= (fullV - BATTERY_FULL_TOLERANCE_V)) {
    chargeStage = STAGE_HOLDING;
  } else if (rampCeilingVoltage >= fullV) {
    chargeStage = STAGE_BULK;
  }

  // CC/CV regulation overrides fixed duty when limits are hit.
  if (batteryCurrent > MAX_CHARGE_CURRENT_A) {
    duty -= CHARGE_CONTROL_CORRECTION_STEP_PERCENT;
  } else if (batteryVoltage > rampCeilingVoltage) {
    duty -= CHARGE_CONTROL_CORRECTION_STEP_PERCENT;
  }
  if (duty > MPPT_DUTY_MAX_PERCENT) duty = MPPT_DUTY_MAX_PERCENT;
  if (duty < MPPT_DUTY_MIN_PERCENT) duty = MPPT_DUTY_MIN_PERCENT;

  pwmSetDuty(duty);
  perturbCount++;

  if (batteryCurrent > 0.1f) hasSeenCurrentFlow = true;

  if (chargeStage == STAGE_HOLDING &&
      hasSeenCurrentFlow &&
      batteryVoltage >= (fullV - BATTERY_FULL_TOLERANCE_V) &&
      batteryCurrent <= TERMINATION_CURRENT_A &&
      (millis() - chargingStartTime) >= MIN_CHARGE_RUNTIME_BEFORE_TERMINATION_MS) {
    stopCharging("Charging complete -- battery full (Fixed PWM mode, " +
                 String(CHEMISTRY_NAMES[selectedChemistry]) + ")", STOP_COMPLETE);
  }
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
    if (operatingMode == MODE_MPPT) {
      mpptStep();
    } else {
      fixedPwmStep();
    }
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

void handleState() {
  static char buf[1600];
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
    "\"fixedPwmDutyPercent\":%.2f,"
    "\"autoRestartEnabled\":%s,"
    "\"autoRestartEligible\":%s,"
    "\"autoRestartCountdown\":%u"
    "}",
    pvVoltage, pvCurrent, pvPower,
    batteryVoltage, batteryCurrent, batteryPower,
    dutyPercent, MPPT_DUTY_MIN_PERCENT, MPPT_DUTY_MAX_PERCENT,
    (int)perturbDirection,
    mpptRunning ? "true" : "false",
    fullV + MPPT_MIN_PV_HEADROOM_START_V,
    lastFeedforwardDutyPercent,
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
    batteryTempC,
    ntcValid ? "true" : "false",
    perturbCount,
    pvEnergyWh, batteryEnergyWh,
    efficiencyValid ? instantEfficiency    : 0.0f,
    pvEnergyWh > 0.001f ? cumulativeEfficiency : 0.0f,
    efficiencyValid  ? "true" : "false",
    effSuspicious    ? "true" : "false",
    sessionId,
    operatingMode == MODE_MPPT ? "MPPT" : "PWM",
    fixedPwmDutyPercent,
    autoRestartEnabled ? "true" : "false",
    (lastStopType == STOP_AUTO_PV) ? "true" : "false",
    (!mpptRunning && autoRestartEnabled && lastStopType == STOP_AUTO_PV)
      ? (millis() - lastAutoStopTime >= AUTO_RESTART_INTERVAL_MS
          ? 0u
          : (uint32_t)((AUTO_RESTART_INTERVAL_MS - (millis() - lastAutoStopTime)) / 1000))
      : 0u
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
  } else if (m == "PWM") {
    operatingMode = MODE_PWM;
    server.send(200, "application/json", "{\"ok\":true,\"mode\":\"PWM\"}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Invalid mode (use MPPT or PWM)\"}");
  }
}

void handleSetFixedDuty() {
  if (mpptRunning) {
    server.send(200, "application/json", "{\"ok\":false,\"reason\":\"Stop charging before changing fixed duty\"}");
    return;
  }
  if (!server.hasArg("duty")) {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"Missing duty parameter\"}");
    return;
  }
  float d = server.arg("duty").toFloat();
  if (d < PWM_DUTY_HARD_MIN_PERCENT || d > PWM_DUTY_HARD_MAX_PERCENT) {
    server.send(200, "application/json", "{\"ok\":false,\"reason\":\"Duty out of range ("
                + String(PWM_DUTY_HARD_MIN_PERCENT, 0) + "–"
                + String(PWM_DUTY_HARD_MAX_PERCENT, 0) + "%)\"}");
    return;
  }
  fixedPwmDutyPercent = d;
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
  WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP started. Connect to Wi-Fi \"");
  Serial.print(AP_SSID);
  Serial.print("\" and open http://");
  Serial.println(apIP);

  // ---- HTTP routes ----
  server.serveStatic("/", LittleFS, "/index.html");
  server.on("/state",     HTTP_GET,  handleState);
  server.on("/start",     HTTP_POST, handleStart);
  server.on("/stop",      HTTP_POST, handleStop);
  server.on("/chemistry", HTTP_POST, handleSetChemistry);
  server.on("/cellcount",    HTTP_POST, handleSetCellCount);
  server.on("/resetenergy",  HTTP_POST, handleResetEnergy);
  server.on("/autorestart",  HTTP_POST, handleSetAutoRestart);
  server.on("/mode",         HTTP_POST, handleSetMode);
  server.on("/fixedduty",    HTTP_POST, handleSetFixedDuty);
  server.onNotFound(handleNotFound);
  server.begin();

  sampleSensors();  // seed EMA from real readings before loop starts
  pwmInit(MPPT_DUTY_START_PERCENT);
  gatesForceOff();  // boot in off state -- PWM starts only when Start is pressed
}

void loop() {
  server.handleClient();
  serviceSensors();
  serviceCharger();
}
