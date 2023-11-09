#include <Arduino.h>

#include "conf/datatypes.h"
#include "conf/confparser.h"
#include "conf/buffer.h"
#include "conf/conf_default.h"

#include <math.h>
#include <string.h>

// Acceleration average
#define ACCEL_ARRAY_SIZE 40

// ADC Hand-Press Scale Factor (Accomdate lighter presses than what's needed for engagement by foot)
#define ADC_HAND_PRESS_SCALE 0.8

// Return the sign of the argument. -1.0 if negative, 1.0 if zero or positive.
#define SIGN(x)				(((x) < 0.0) ? -1.0 : 1.0)

#define DEG2RAD_f(deg)		((deg) * (float)(M_PI / 180.0))
#define RAD2DEG_f(rad) 		((rad) * (float)(180.0 / M_PI))

#define UNUSED(x) (void)(x)

#define PIN_ADC1 1
#define PIN_ADC2 2


// Data type
typedef enum {
	STARTUP = 0,
	RUNNING = 1,
	RUNNING_TILTBACK = 2,
	RUNNING_WHEELSLIP = 3,
	RUNNING_UPSIDEDOWN = 4,
	RUNNING_FLYWHEEL = 5,   // we remain in "RUNNING" state in flywheel mode,
	                        // but then report "RUNNING_FLYWHEEL" in rt data
	FAULT_ANGLE_PITCH = 6,	// skipped 5 for compatibility
	FAULT_ANGLE_ROLL = 7,
	FAULT_SWITCH_HALF = 8,
	FAULT_SWITCH_FULL = 9,
	FAULT_DUTY = 10, 		// unused but kept for compatibility
	FAULT_STARTUP = 11,
	FAULT_REVERSE = 12,
	FAULT_QUICKSTOP = 13,
	DISABLED = 15
} FloatState;

typedef enum {
	BEEP_NONE = 0,
	BEEP_LV = 1,
	BEEP_HV = 2,
	BEEP_TEMPFET = 3,
	BEEP_TEMPMOT = 4,
	BEEP_CURRENT = 5,
	BEEP_DUTY = 6,
	BEEP_SENSORS = 7,
	BEEP_LOWBATT = 8,
	BEEP_IDLE = 9,
	BEEP_ERROR = 10
} BeepReason;

typedef enum {
	CENTERING = 0,
	REVERSESTOP,
	TILTBACK_NONE,
	TILTBACK_DUTY,
	TILTBACK_HV,
	TILTBACK_LV,
	TILTBACK_TEMP
} SetpointAdjustmentType;

typedef enum {
	OFF = 0,
	HALF,
	ON
} SwitchState;

typedef struct {
	float a0, a1, a2, b1, b2;
	float z1, z2;
} Biquad;

typedef enum {
	BQ_LOWPASS,
	BQ_HIGHPASS
} BiquadType;

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
	float integralFBx;
	float integralFBy;
	float integralFBz;
	float accMagP;
	int initialUpdateDone;

	// Parameters
	float acc_confidence_decay;
	float kp;
	float ki;
	float beta;
} ATTITUDE_INFO;

// This is all persistent state of the application, which will be allocated in init. It
// is put here because variables can only be read-only when this program is loaded
// in flash without virtual memory in RAM (as all RAM already is dedicated to the
// main firmware and managed from there). This is probably the main limitation of
// loading applications in runtime, but it is not too bad to work around.
typedef struct {
	// lib_thread thread; // Balance Thread
	float_config float_conf;

	// Firmware version, passed in from Lisp
	int fw_version_major, fw_version_minor, fw_version_beta;

	// Buzzer
	int beep_num_left;
	int beep_duration;
	int beep_countdown;
	int beep_reason;
	bool buzzer_enabled;

	// Config values
	float loop_time_seconds;
	unsigned int start_counter_clicks, start_counter_clicks_max;
	float startup_pitch_trickmargin, startup_pitch_tolerance;
	float startup_step_size;
	float tiltback_duty_step_size, tiltback_hv_step_size, tiltback_lv_step_size, tiltback_return_step_size;
	float torquetilt_on_step_size, torquetilt_off_step_size, turntilt_step_size;
	float tiltback_variable, tiltback_variable_max_erpm, noseangling_step_size, inputtilt_ramped_step_size, inputtilt_step_size;
	float mc_max_temp_fet, mc_max_temp_mot;
	float mc_current_max, mc_current_min, max_continuous_current;
	float surge_angle, surge_angle2, surge_angle3, surge_adder;
	bool surge_enable;
	bool current_beeping;
	bool duty_beeping;

	// Feature: True Pitch
	ATTITUDE_INFO m_att_ref;

	// Runtime values read from elsewhere
	float pitch_angle, last_pitch_angle, roll_angle, abs_roll_angle, abs_roll_angle_sin, last_gyro_y;
 	float true_pitch_angle;
	float gyro[3];
	float duty_cycle, abs_duty_cycle, duty_smooth;
	float erpm, abs_erpm, avg_erpm;
	float motor_current;
	float adc1, adc2;
	float throttle_val;
	float max_duty_with_margin;
	SwitchState switch_state;

	// Feature: ATR (Adaptive Torque Response)
	float atr_on_step_size, atr_off_step_size;
	float acceleration, last_erpm;
	float accel_gap;
	float accelhist[ACCEL_ARRAY_SIZE];
	float accelavg;
	int accelidx;
	int direction_counter;
	bool braking;

	// Feature: Turntilt
	float last_yaw_angle, yaw_angle, abs_yaw_change, last_yaw_change, yaw_change, yaw_aggregate;
	float turntilt_boost_per_erpm, yaw_aggregate_target;

	// Rumtime state values
	FloatState state;
	float proportional;
	float pid_prop, pid_integral, pid_rate, pid_mod;
	float last_proportional, abs_proportional;
	float pid_value;
	float setpoint, setpoint_target, setpoint_target_interpolated;
	float applied_booster_current;
	float noseangling_interpolated, inputtilt_interpolated;
	float filtered_current;
	float torquetilt_target, torquetilt_interpolated;
	float atr_filtered_current, atr_target, atr_interpolated;
	float torqueresponse_interpolated;
	Biquad atr_current_biquad;
	float braketilt_factor, braketilt_target, braketilt_interpolated;
	float turntilt_target, turntilt_interpolated;
	SetpointAdjustmentType setpointAdjustmentType;
	float current_time, last_time, diff_time, loop_overshoot; // Seconds
	float disengage_timer, nag_timer; // Seconds
	float idle_voltage;
	float filtered_loop_overshoot, loop_overshoot_alpha, filtered_diff_time;
	float fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer, fault_switch_half_timer; // Seconds
	float motor_timeout_seconds;
	float brake_timeout; // Seconds
	float wheelslip_timer, wheelslip_end_timer, overcurrent_timer, tb_highvoltage_timer;
	float switch_warn_buzz_erpm;
	float quickstop_erpm;
	bool traction_control;

	// PID Brake Scaling
	float kp_brake_scale; // Used for brakes when riding forwards, and accel when riding backwards
	float kp2_brake_scale;
	float kp_accel_scale; // Used for accel when riding forwards, and brakes when riding backwards
	float kp2_accel_scale;

	// Darkride aka upside down mode:
	bool is_upside_down;			// the board is upside down
	bool is_upside_down_started;	// dark ride has been engaged
	bool enable_upside_down;		// dark ride mode is enabled (10 seconds after fault)
	float delay_upside_down_fault;
	float darkride_setpoint_correction;

	// Feature: Flywheel
	bool is_flywheel_mode, flywheel_abort, flywheel_allow_abort;
	float flywheel_pitch_offset, flywheel_roll_offset, flywheel_konami_timer, flywheel_konami_pitch;
	int flywheel_konami_state;

	// Feature: Handtest
	bool do_handtest;

	// Feature: Reverse Stop
	float reverse_stop_step_size, reverse_tolerance, reverse_total_erpm;
	float reverse_timer;

	// Feature: Soft Start
	float softstart_pid_limit, softstart_ramp_step_size;

	// Brake Amp Rate Limiting:
	float pid_brake_increment;

	// Odometer
	float odo_timer;
	int odometer_dirty;
	uint64_t odometer;

	// Feature: RC Move (control via app while idle)
	int rc_steps;
	int rc_counter;
	float rc_current_target;
	float rc_current;

	// Log values
	float float_setpoint, float_atr, float_braketilt, float_torquetilt, float_turntilt, float_inputtilt;
	float float_expected_acc, float_measured_acc, float_acc_diff;

	// Debug values
	int debug_render_1, debug_render_2;
	int debug_sample_field, debug_sample_count, debug_sample_index;
	int debug_experiment_1, debug_experiment_2, debug_experiment_3, debug_experiment_4, debug_experiment_5, debug_experiment_6;
} data;

void brake(data *d);
void set_current(data *d, float current);
void flywheel_stop(data *d);
void cmd_flywheel_toggle(data *d, unsigned char *cfg, int len);
bool flywheel_konami_check(data *d);
bool flywheel_konami_step(data *d, int input);

/**
 * BUZZER / BEEPER on Servo Pin
 */
const int buzzer_pin = 0; //****

void buzzer_init(void);
void buzzer_update(data *d);
void buzzer_enable(data *d, bool enable);
void beep_alert(data *d, int num_beeps, bool longbeep);
void beep_off(data *d, bool force);
void beep_on(data *d, bool force);
float biquad_process(Biquad *biquad, float in);
void biquad_config(Biquad *biquad, BiquadType type, float Fc);
void biquad_reset(Biquad *biquad);
void app_init(data *d);
void configure(data *d);
void reset_vars(data *d);
void do_rc_move(data *d);
float get_setpoint_adjustment_step_size(data *d);
SwitchState check_adcs(data *d);
bool check_faults(data *d);
void calculate_setpoint_target(data *d);
void calculate_setpoint_interpolated(data *d);
void add_surge(data *d);
void calculate_torqueresponse_interpolated(data *d);
void apply_noseangling(data *d);
void apply_inputtilt(data *d); // Input Tiltback
void apply_torquetilt(data *d);
void apply_turntilt(data *d);
void brake(data *d);
void set_current(data *d, float current);
void float_thd(void *arg);
void read_cfg_from_eeprom(data *d);
float app_float_get_debug(data *d, int index);
void cmd_tune_defaults(data *d);
void get_imu_data();

float mc_get_input_voltage_filtered(void);
float mc_temp_fet_filtered(void);
float mc_temp_motor_filtered(void);
void mc_set_brake_current(float current);
bool should_terminate(void);
float ahrs_get_roll(ATTITUDE_INFO* att_ref);
float ahrs_get_pitch(ATTITUDE_INFO* att_ref);
float ahrs_get_yaw(ATTITUDE_INFO* att_ref);
float imu_get_pitch(void);
void imu_get_gyro(float *gyro);
float mc_get_rpm(void);
bool imu_startup_done(void);

void EXT_BUZZER_OFF(void);
void EXT_BUZZER_ON(void);
void init(void);