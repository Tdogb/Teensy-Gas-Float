#include <Arduino.h>

#include "conf/datatypes.h"
#include "conf/confparser.h"
#include "conf/buffer.h"
#include "conf/conf_default.h"

#include <math.h>
#include <string.h>

#include <ODriveArduino.h>
#include <Servo.h>
#include <Chrono.h>

#include "float-imu.h"
#include "float-vesc.h"

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
float mc_get_duty_cycle_now(void);
bool should_terminate(void);
float ahrs_get_roll(ATTITUDE_INFO* att_ref);
float ahrs_get_pitch(ATTITUDE_INFO* att_ref);
float ahrs_get_yaw(ATTITUDE_INFO* att_ref);
float imu_get_pitch(void);
void imu_get_gyro(float *gyro);
void update_imu(void);
float mc_get_rpm(void);
bool imu_startup_done(void);
void EXT_BUZZER_OFF(void);
void EXT_BUZZER_ON(void);
float get_current(void);
float l_current_max = 75;
float l_current_min = 0;
Servo engine_throttle_servo;
float_imu imu;
float_vesc esc;

float gear_ratio = 0.1142857143

/**
 * USE HASH TABLES
*/
float engine_rpms[25] = 
{3500,         3666.66666667, 3833.33333333, 4000,         4166.66666667,
 4333.33333333, 4500,         4666.66666667, 4833.33333333, 5000,
 5166.66666667, 5333.33333333, 5500,5666.66666667, 5833.33333333,
 6000,        6166.66666667, 6333.33333333, 6500,         6666.66666667,
 6833.33333333, 7000,         7166.66666667, 7333.33333333, 7500,       };

float engine_throttles[20] = 
{0,         0.05263158, 0.10526316, 0.15789474, 0.21052632, 0.26315789,
 0.31578947, 0.36842105, 0.42105263, 0.47368421, 0.52631579, 0.57894737,
 0.63157895, 0.68421053, 0.73684211, 0.78947368, 0.84210526, 0.89473684,
 0.94736842, 1,        };

float engine_torques[20][25] = {{0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2},
						  {0.20787061403508775, 0.21328947368421053, 0.2183618421052632, 0.22308771929824567, 0.227467105263158, 0.23149999999999998, 0.23518640350877193, 0.2385263157894737, 0.2415197368421053, 0.24416666666666673, 0.24646710526315793, 0.24842105263157896, 0.25002850877192984, 0.25128947368421056, 0.25220394736842106, 0.2527719298245614, 0.25299342105263156, 0.2528684210526316, 0.25239692982456147, 0.2515789473684211, 0.25041447368421055, 0.24890350877192985, 0.24704605263157894, 0.2448421052631579, 0.24229166666666668},
						  {0.21574122807017543, 0.22657894736842096, 0.23672368421052628, 0.24617543859649127, 0.25493421052631593, 0.2629999999999999, 0.2703728070175438, 0.27705263157894733, 0.2830394736842105, 0.2883333333333334, 0.2929342105263158, 0.29684210526315785, 0.30005701754385966, 0.302578947368421, 0.30440789473684204, 0.30554385964912273, 0.3059868421052631, 0.3057368421052631, 0.30479385964912287, 0.3031578947368422, 0.3008289473684211, 0.29780701754385963, 0.2940921052631578, 0.2896842105263157, 0.2845833333333333},
						  {0.22361184210526314, 0.23986842105263145, 0.2550855263157894, 0.26926315789473687, 0.28240131578947386, 0.2944999999999999, 0.3055592105263156, 0.31557894736842107, 0.32455921052631576, 0.3325, 0.3394013157894737, 0.34526315789473677, 0.3500855263157895, 0.3538684210526315, 0.356611842105263, 0.35831578947368414, 0.35898026315789466, 0.35860526315789454, 0.3571907894736843, 0.35473684210526335, 0.3512434210526316, 0.34671052631578947, 0.34113815789473667, 0.33452631578947356, 0.32687499999999997},
						  {0.2314824561403509, 0.25315789473684197, 0.2734473684210526, 0.29235087719298253, 0.3098684210526318, 0.32599999999999985, 0.3407456140350876, 0.3541052631578947, 0.366078947368421, 0.37666666666666676, 0.3858684210526316, 0.39368421052631575, 0.40011403508771937, 0.40515789473684205, 0.408815789473684, 0.4110877192982455, 0.41197368421052627, 0.4114736842105261, 0.4095877192982458, 0.4063157894736844, 0.40165789473684216, 0.3956140350877193, 0.38818421052631563, 0.3793684210526314, 0.36916666666666664},
						  {0.2393530701754386, 0.26644736842105243, 0.2918092105263157, 0.3154385964912282, 0.3373355263157898, 0.3574999999999998, 0.3759320175438594, 0.39263157894736844, 0.40759868421052625, 0.42083333333333345, 0.4323355263157895, 0.44210526315789467, 0.45014254385964914, 0.45644736842105255, 0.461019736842105, 0.46385964912280686, 0.4649671052631578, 0.4643421052631576, 0.4619846491228073, 0.4578947368421055, 0.4520723684210527, 0.4445175438596492, 0.43523026315789454, 0.42421052631578926, 0.4114583333333333},
						  {0.24722368421052632, 0.27973684210526295, 0.31017105263157885, 0.3385263157894738, 0.36480263157894777, 0.3889999999999998, 0.4111184210526313, 0.4311578947368421, 0.44911842105263144, 0.4650000000000001, 0.4788026315789474, 0.4905263157894736, 0.500171052631579, 0.507736842105263, 0.513223684210526, 0.5166315789473682, 0.5179605263157894, 0.5172105263157891, 0.5143815789473687, 0.5094736842105266, 0.5024868421052632, 0.493421052631579, 0.4822763157894734, 0.46905263157894705, 0.45375},
						  {0.255094298245614, 0.29302631578947336, 0.328532894736842, 0.3616140350877194, 0.3922697368421057, 0.42049999999999976, 0.4463048245614032, 0.46968421052631576, 0.49063815789473675, 0.5091666666666668, 0.5252697368421053, 0.5389473684210525, 0.5501995614035088, 0.5590263157894735, 0.565427631578947, 0.5694035087719296, 0.5709539473684209, 0.5700789473684207, 0.5667785087719301, 0.5610526315789477, 0.5529013157894738, 0.5423245614035088, 0.5293223684210524, 0.513894736842105, 0.4960416666666666},
						  {0.26296491228070173, 0.3063157894736839, 0.34689473684210514, 0.38470175438596504, 0.41973684210526363, 0.45199999999999974, 0.48149122807017514, 0.5082105263157894, 0.5321578947368419, 0.5533333333333335, 0.5717368421052631, 0.5873684210526314, 0.6002280701754387, 0.610315789473684, 0.6176315789473681, 0.6221754385964909, 0.6239473684210525, 0.6229473684210522, 0.6191754385964916, 0.6126315789473689, 0.6033157894736842, 0.5912280701754387, 0.5763684210526312, 0.5587368421052628, 0.5383333333333332},
						  {0.27083552631578944, 0.3196052631578944, 0.3652565789473683, 0.4077894736842107, 0.4472039473684216, 0.4834999999999997, 0.516677631578947, 0.5467368421052631, 0.5736776315789472, 0.5975000000000001, 0.618203947368421, 0.6357894736842103, 0.6502565789473684, 0.6616052631578946, 0.669835526315789, 0.6749473684210523, 0.6769407894736841, 0.6758157894736837, 0.6715723684210531, 0.6642105263157899, 0.6537302631578948, 0.6401315789473685, 0.6234144736842101, 0.6035789473684207, 0.580625},
						  {0.27870614035087715, 0.33289473684210485, 0.38361842105263144, 0.4308771929824563, 0.47467105263157955, 0.5149999999999997, 0.5518640350877189, 0.5852631578947368, 0.6151973684210525, 0.6416666666666668, 0.6646710526315789, 0.6842105263157894, 0.7002850877192983, 0.712894736842105, 0.72203947368421, 0.7277192982456138, 0.7299342105263156, 0.7286842105263152, 0.7239692982456145, 0.715789473684211, 0.7041447368421054, 0.6890350877192983, 0.670460526315789, 0.6484210526315786, 0.6229166666666666},
						  {0.2865767543859649, 0.3461842105263153, 0.4019802631578946, 0.4539649122807019, 0.5021381578947376, 0.5464999999999997, 0.5870504385964909, 0.6237894736842106, 0.6567171052631579, 0.6858333333333336, 0.7111381578947369, 0.7326315789473683, 0.7503135964912282, 0.7641842105263157, 0.7742434210526311, 0.7804912280701752, 0.7829276315789473, 0.7815526315789468, 0.7763662280701761, 0.7673684210526323, 0.754559210526316, 0.7379385964912283, 0.717506578947368, 0.6932631578947365, 0.6652083333333333},
						  {0.29444736842105257, 0.35947368421052583, 0.42034210526315774, 0.4770526315789475, 0.5296052631578955, 0.5779999999999995, 0.6222368421052626, 0.6623157894736842, 0.6982368421052629, 0.7300000000000002, 0.7576052631578948, 0.7810526315789471, 0.800342105263158, 0.815473684210526, 0.826447368421052, 0.8332631578947365, 0.8359210526315787, 0.8344210526315783, 0.8287631578947374, 0.8189473684210533, 0.8049736842105264, 0.786842105263158, 0.7645526315789468, 0.7381052631578942, 0.7074999999999999},
						  {0.30231798245614033, 0.3727631578947363, 0.4387039473684209, 0.5001403508771931, 0.5570723684210533, 0.6094999999999995, 0.6574232456140344, 0.7008421052631577, 0.7397565789473681, 0.7741666666666668, 0.8040723684210526, 0.8294736842105259, 0.8503706140350876, 0.8667631578947365, 0.878651315789473, 0.8860350877192977, 0.8889144736842102, 0.8872894736842097, 0.8811600877192988, 0.8705263157894743, 0.8553881578947369, 0.8357456140350877, 0.8115986842105257, 0.782947368421052, 0.7497916666666665},
						  {0.31018859649122804, 0.38605263157894676, 0.45706578947368404, 0.5232280701754388, 0.5845394736842114, 0.6409999999999996, 0.6926096491228064, 0.7393684210526316, 0.7812763157894735, 0.8183333333333336, 0.8505394736842106, 0.8778947368421051, 0.9003991228070176, 0.9180526315789471, 0.9308552631578941, 0.9388070175438592, 0.9419078947368419, 0.9401578947368414, 0.9335570175438603, 0.9221052631578955, 0.9058026315789476, 0.8846491228070177, 0.8586447368421047, 0.82778947368421, 0.7920833333333333},
						  {0.3180592105263158, 0.39934210526315733, 0.47542763157894724, 0.5463157894736844, 0.6120065789473693, 0.6724999999999995, 0.7277960526315783, 0.7778947368421052, 0.8227960526315787, 0.8625000000000003, 0.8970065789473685, 0.926315789473684, 0.9504276315789475, 0.9693421052631577, 0.9830592105263152, 0.9915789473684207, 0.9949013157894735, 0.993026315789473, 0.9859539473684218, 0.9736842105263166, 0.9562171052631581, 0.9335526315789475, 0.9056907894736836, 0.8726315789473678, 0.834375},
						  {0.32592982456140346, 0.41263157894736774, 0.4937894736842103, 0.56940350877193, 0.6394736842105272, 0.7039999999999994, 0.7629824561403502, 0.8164210526315788, 0.8643157894736839, 0.906666666666667, 0.9434736842105262, 0.9747368421052628, 1.0004561403508774, 1.0206315789473681, 1.0352631578947362, 1.044350877192982, 1.047894736842105, 1.0458947368421045, 1.0383508771929832, 1.0252631578947378, 1.0066315789473685, 0.9824561403508772, 0.9527368421052624, 0.9174736842105256, 0.8766666666666665},
						  {0.33380043859649117, 0.4259210526315782, 0.5121513157894735, 0.5924912280701757, 0.6669407894736852, 0.7354999999999994, 0.7981688596491221, 0.8549473684210526, 0.9058355263157892, 0.9508333333333336, 0.9899407894736842, 1.0231578947368418, 1.050484649122807, 1.0719210526315786, 1.087467105263157, 1.0971228070175434, 1.1008881578947365, 1.0987631578947359, 1.0907478070175447, 1.0768421052631587, 1.057046052631579, 1.0313596491228072, 0.9997828947368413, 0.9623157894736835, 0.9189583333333332},
						  {0.34167105263157893, 0.4392105263157888, 0.5305131578947366, 0.6155789473684213, 0.6944078947368432, 0.7669999999999993, 0.833355263157894, 0.8934736842105262, 0.9473552631578944, 0.9950000000000003, 1.036407894736842, 1.0715789473684207, 1.100513157894737, 1.1232105263157892, 1.139671052631578, 1.1498947368421046, 1.1538815789473682, 1.1516315789473675, 1.1431447368421062, 1.1284210526315799, 1.1074605263157897, 1.080263157894737, 1.0468289473684202, 1.0071578947368411, 0.9612499999999998},
						  {0.34954166666666664, 0.45249999999999924, 0.5488749999999998, 0.6386666666666669, 0.7218750000000012, 0.7984999999999993, 0.8685416666666659, 0.9319999999999999, 0.9888749999999997, 1.039166666666667, 1.082875, 1.1199999999999997, 1.1505416666666668, 1.1744999999999997, 1.1918749999999991, 1.202666666666666, 1.2068749999999997, 1.204499999999999, 1.1955416666666676, 1.180000000000001, 1.1578750000000002, 1.1291666666666669, 1.0938749999999993, 1.0519999999999992, 1.0035416666666666}};