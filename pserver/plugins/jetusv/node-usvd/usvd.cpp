#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <float.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <wchar.h>
#include <limits.h>
#include <dirent.h>

#include "usvd.h"

#include <pthread.h>
#include "nlohmann/json.hpp"
#include <mat4/identity.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static USVD_PWM_CALLBACK lg_pwm_callback = NULL;
static void *lg_pwm_callback_arg = NULL;

#define LIGHT_NUM 2
#define MOTOR_NUM 4
static float lg_motor_center = 1500;
static float lg_motor_margin = 25;
static float lg_motor_range = 100;
#define MOTOR_BASE(value) lg_motor_center + lg_motor_margin * ((value < 0.5 && value > -0.5) ? 0 : (value > 0) ? 1 : -1)

static int lg_light_id[LIGHT_NUM] = { 4, 5 };
static int lg_motor_id[MOTOR_NUM] = { 0, 1, 2, 3 };

static float lg_light_value[LIGHT_NUM] = { 0, 0 };
static float lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_motor_dir[4] = { 1, 1, 1, 1 };

static float lg_light_strength = 0; //0 to 100
static float lg_thrust = 0; //-100 to 100
static float lg_rudder = 0; //-100 to 100
static float lg_target_heading = 0; //-180 to 180
static float lg_max_rpm = 6;
static int lg_thruster_mode = 1; //0:single, 1:double, 2:quad

#define PID_NUM 2
static bool lg_emergency_mode = false;
static bool lg_lowlevel_control = false;
static bool lg_pid_enabled = false;
static bool lg_heading_lock = true;
static float lg_pid_gain[PID_NUM][3] = { { 1.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } };
static float lg_pid_value[PID_NUM] = { }; //[rpm, heading]
static float lg_delta_pid_target[3][PID_NUM * 2 + 1] = { }; //[history][rpm, rpm_lpf, heading, heading_lpf t]

static void update_pwm() {
	if(lg_pwm_callback == NULL){
		return;
	}

//	{
//		float value = lg_light_value[0];
//		value = pow(value / 100, 3);
//		len = sprintf(cmd, "%d=%f\n", lg_light_id[0], value);
//		write(fd, cmd, len);
//	}
//
//	{
//		float value = lg_light_value[1];
//		value = pow(value / 100, 3);
//		len = sprintf(cmd, "%d=%f\n", lg_light_id[1], value);
//		write(fd, cmd, len);
//	}

	{
		float value = lg_motor_value[0];
		value = lg_motor_dir[0] * (value / 100) * lg_motor_range + MOTOR_BASE(lg_motor_dir[0] * value);
		
		lg_pwm_callback(lg_motor_id[0], value, lg_pwm_callback_arg);
	}

	{
		float value = lg_motor_value[1];
		value = lg_motor_dir[1] * (value / 100) * lg_motor_range + MOTOR_BASE(lg_motor_dir[1] * value);
		
		lg_pwm_callback(lg_motor_id[1], value, lg_pwm_callback_arg);
	}

	{
		float value = lg_motor_value[2];
		value = lg_motor_dir[2] * (value / 100) * lg_motor_range + MOTOR_BASE(lg_motor_dir[2] * value);
		
		lg_pwm_callback(lg_motor_id[2], value, lg_pwm_callback_arg);
	}

	{
		float value = lg_motor_value[3];
		value = lg_motor_dir[3] * (value / 100) * lg_motor_range + MOTOR_BASE(lg_motor_dir[3] * value);
		
		lg_pwm_callback(lg_motor_id[3], value, lg_pwm_callback_arg);
	}
}

static float normalize_angle(float v) {
	v -= floor(v / 360) * 360;
	if (v < -180.0) {
		v += 360.0;
	}
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}

static bool lg_debugdump = false;
static void pid_control_single(float t_s, float north) {
	if (!lg_pid_enabled) {
		lg_motor_value[0] = lg_thrust;
		lg_motor_value[1] = lg_rudder;
		return;
	}

	static float last_t_s = -1;
	if (last_t_s < 0) {
		last_t_s = t_s;
		return;
	}
	float diff_sec = t_s - last_t_s;
	if (diff_sec < 0.01) {
		return;
	}
	last_t_s = t_s;

	static float lpf_gain1 = 0.8;
	static float lpf_gain2 = 0.2;
	static float heading_lpf1 = FLT_MIN;
	static float last_heading_lpf1 = FLT_MIN;
	static float heading_lpf2 = FLT_MIN;
	static float last_heading_lpf2 = FLT_MIN;
	float heading = -north; //clockwise
	if (heading_lpf1 == FLT_MIN) {
		heading_lpf1 = heading;
		last_heading_lpf1 = heading;
		heading_lpf2 = heading;
		last_heading_lpf2 = heading;
		return;
	}
	heading_lpf1 = normalize_angle(heading_lpf1 + normalize_angle(heading - heading_lpf1) * lpf_gain1);
	heading_lpf2 = normalize_angle(heading_lpf2 + normalize_angle(heading - heading_lpf2) * lpf_gain2);

	float rpm_lpf1 = normalize_angle(heading_lpf1 - last_heading_lpf1) / 360 / diff_sec * 60;
	float rpm_lpf2 = normalize_angle(heading_lpf2 - last_heading_lpf2) / 360 / diff_sec * 60;
	float target_rpm = lg_rudder * lg_max_rpm / 100;
	lg_delta_pid_target[0][0] = rpm_lpf1 - target_rpm;
	lg_delta_pid_target[0][1] = rpm_lpf2 - target_rpm;
	lg_delta_pid_target[0][2] = normalize_angle(heading_lpf1 - lg_target_heading);
	lg_delta_pid_target[0][3] = normalize_angle(heading_lpf2 - lg_target_heading);
	lg_delta_pid_target[0][PID_NUM * 2] = t_s;
	last_heading_lpf1 = heading_lpf1;
	last_heading_lpf2 = heading_lpf2;

	if (lg_delta_pid_target[2][PID_NUM * 2] == 0) { //skip
		//increment
		for (int j = 3 - 1; j >= 1; j--) {
			for (int k = 0; k < PID_NUM * 2 + 1; k++) {
				lg_delta_pid_target[j][k] = lg_delta_pid_target[j - 1][k];
			}
		}
		return;
	}

	bool is_angle[PID_NUM] = { false, true };
	for (int k = 0; k < PID_NUM; k++) {
		float p_value = lg_pid_gain[k][0] * lg_delta_pid_target[0][k * 2];
		float diff = (lg_delta_pid_target[0][k * 2 + 1] - lg_delta_pid_target[1][k * 2 + 1]);
		if (is_angle[k]) {
			diff = normalize_angle(diff);
		}
		float d_value = lg_pid_gain[k][2] * diff / diff_sec;
		float delta_value = p_value + d_value;
		lg_pid_value[k] = delta_value;
	}
	//increment
	for (int j = 3 - 1; j >= 1; j--) {
		for (int k = 0; k < PID_NUM * 2 + 1; k++) {
			lg_delta_pid_target[j][k] = lg_delta_pid_target[j - 1][k];
		}
	}
	{		//limit
		lg_pid_value[0] = MIN(MAX(lg_pid_value[0], -200), 200);		//rpm to thruster differencial
		lg_pid_value[1] = MIN(MAX(lg_pid_value[1], -200), 200);		//heading to thruster differencial
	}

	if (lg_debugdump) {
		printf("vehicle t=%.3fs: rpm=%.3f, %.3f, %.3f : heading=%.3f, %.3f, %.3f\n", diff_sec, rpm_lpf1, target_rpm, lg_pid_value[0], heading_lpf1, lg_target_heading, lg_pid_value[1]);
	}

	{		//aply
		float lpf_gain = 0.5;
		float value = (lg_heading_lock ? lg_pid_value[1] : lg_pid_value[0]);
		lg_motor_value[0] = lg_motor_value[0] * (1 - lpf_gain) + lg_thrust * lpf_gain;
		lg_motor_value[1] = lg_motor_value[1] * (1 - lpf_gain) + value * lpf_gain;
		return;
	}
}
static void pid_control_double(float t_s, float north) {
	if (!lg_pid_enabled) {
		float gain = 2.0;
		float rudder = lg_target_heading / 180;
		float value_r = lg_thrust + gain*rudder / 2;
		float value_l = lg_thrust - gain*rudder / 2;
		if (lg_emergency_mode) {
			lg_motor_value[0] = 0;
			lg_motor_value[1] = 0;
			lg_motor_value[2] = value_r;
			lg_motor_value[3] = value_l;
		} else {
			lg_motor_value[0] = value_r;
			lg_motor_value[1] = value_l;
			lg_motor_value[2] = 0;
			lg_motor_value[3] = 0;
		}
		return;
	}

	static float last_t_s = -1;
	if (last_t_s < 0) {
		last_t_s = t_s;
		return;
	}
	float diff_sec = t_s - last_t_s;
	if (diff_sec < 0.01) {
		return;
	}
	last_t_s = t_s;

	static float lpf_gain1 = 0.8;
	static float lpf_gain2 = 0.2;
	static float heading_lpf1 = FLT_MIN;
	static float last_heading_lpf1 = FLT_MIN;
	static float heading_lpf2 = FLT_MIN;
	static float last_heading_lpf2 = FLT_MIN;
	float heading = -north; //clockwise
	if (heading_lpf1 == FLT_MIN) {
		heading_lpf1 = heading;
		last_heading_lpf1 = heading;
		heading_lpf2 = heading;
		last_heading_lpf2 = heading;
		return;
	}
	heading_lpf1 = normalize_angle(heading_lpf1 + normalize_angle(heading - heading_lpf1) * lpf_gain1);
	heading_lpf2 = normalize_angle(heading_lpf2 + normalize_angle(heading - heading_lpf2) * lpf_gain2);

	float rpm_lpf1 = normalize_angle(heading_lpf1 - last_heading_lpf1) / 360 / diff_sec * 60;
	float rpm_lpf2 = normalize_angle(heading_lpf2 - last_heading_lpf2) / 360 / diff_sec * 60;
	float target_rpm = lg_rudder * lg_max_rpm / 100;
	lg_delta_pid_target[0][0] = rpm_lpf1 - target_rpm;
	lg_delta_pid_target[0][1] = rpm_lpf2 - target_rpm;
	lg_delta_pid_target[0][2] = normalize_angle(heading_lpf1 - lg_target_heading);
	lg_delta_pid_target[0][3] = normalize_angle(heading_lpf2 - lg_target_heading);
	lg_delta_pid_target[0][PID_NUM * 2] = t_s;
	last_heading_lpf1 = heading_lpf1;
	last_heading_lpf2 = heading_lpf2;

	if (lg_delta_pid_target[2][PID_NUM * 2] == 0) { //skip
		//increment
		for (int j = 3 - 1; j >= 1; j--) {
			for (int k = 0; k < PID_NUM * 2 + 1; k++) {
				lg_delta_pid_target[j][k] = lg_delta_pid_target[j - 1][k];
			}
		}
		return;
	}

	bool is_angle[PID_NUM] = { false, true };
	for (int k = 0; k < PID_NUM; k++) {
		float p_value = lg_pid_gain[k][0] * lg_delta_pid_target[0][k * 2];
		float diff = (lg_delta_pid_target[0][k * 2 + 1] - lg_delta_pid_target[1][k * 2 + 1]);
		if (is_angle[k]) {
			diff = normalize_angle(diff);
		}
		float d_value = lg_pid_gain[k][2] * diff / diff_sec;
		float delta_value = p_value + d_value;
		lg_pid_value[k] = delta_value;
	}
	//increment
	for (int j = 3 - 1; j >= 1; j--) {
		for (int k = 0; k < PID_NUM * 2 + 1; k++) {
			lg_delta_pid_target[j][k] = lg_delta_pid_target[j - 1][k];
		}
	}
	{		//limit
		lg_pid_value[0] = MIN(MAX(lg_pid_value[0], -200), 200);		//rpm to thruster differencial
		lg_pid_value[1] = MIN(MAX(lg_pid_value[1], -200), 200);		//heading to thruster differencial
	}

	if (lg_debugdump) {
		printf("vehicle t=%.3fs: rpm=%.3f, %.3f, %.3f : heading=%.3f, %.3f, %.3f\n", diff_sec, rpm_lpf1, target_rpm, lg_pid_value[0], heading_lpf1, lg_target_heading, lg_pid_value[1]);
	}

	{		//aply
		float lpf_gain = 0.5;
		float value = (lg_heading_lock ? lg_pid_value[1] : lg_pid_value[0]);
		float value_r = lg_motor_value[0] * (1 - lpf_gain) + (lg_thrust + value / 2) * lpf_gain;
		float value_l = lg_motor_value[1] * (1 - lpf_gain) + (lg_thrust - value / 2) * lpf_gain;
		if (lg_emergency_mode) {
			lg_motor_value[0] = 0;
			lg_motor_value[1] = 0;
			lg_motor_value[2] = value_r;
			lg_motor_value[3] = value_l;
		} else {
			lg_motor_value[0] = value_r;
			lg_motor_value[1] = value_l;
			lg_motor_value[2] = 0;
			lg_motor_value[3] = 0;
		}
		return;
	}
}

static void pid_control_quad(float t_s, float north) {
	return;
}

void usvd_init(const char *config_json){
	{//init pwm
		memset(lg_motor_value, 0, sizeof(lg_motor_value));
		update_pwm();
	}
}

void usvd_set_pwm_callback(USVD_PWM_CALLBACK callback, void *arg){
	lg_pwm_callback = callback;
	lg_pwm_callback_arg = arg;
}

void usvd_poll(float north, float t_s) {

	if (lg_lowlevel_control) {
		return;
	}

	//cal
	//trancate min max
	lg_light_strength = MIN(MAX(lg_light_strength, 0), 100);
	lg_light_value[0] = lg_light_strength;
	lg_light_value[1] = lg_light_strength;

	//trancate min max
	lg_thrust = MIN(MAX(lg_thrust, -100), 100);
	lg_rudder = MIN(MAX(lg_rudder, -100), 100);
	lg_target_heading = normalize_angle(lg_target_heading);

	switch (lg_thruster_mode) {
	case 0:
		pid_control_single(t_s, north);
		break;
	case 1:
		pid_control_double(t_s, north);
		break;
	case 2:
		pid_control_quad(t_s, north);
		break;
	}
	update_pwm();
}

int usvd_command(const char *cmd) {
	if (cmd == NULL) {
		//do nothing
	} else if (strcmp(cmd, "set_thrust") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v1, v2, v3, v4;
			int num = sscanf(param, "%f,%f,%f,%f", &v1, &v2, &v3, &v4);

			if (num >= 1) {
				lg_thrust = v1;
			}
			if (num >= 2 && !lg_heading_lock) {
				lg_rudder = v2;
			}
			if (num >= 3) {
				lg_target_heading = v3;
			}
			if (num >= 4) {
				lg_pid_enabled = (v4 != 0);
			}
			if (lg_debugdump) {
				printf("%s : completed\n", cmd);
			}
		}
	} else if (strcmp(cmd, "set_thruster_mode") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v;
			int num = sscanf(param, "%f", &v);

			if (num >= 1) {
				lg_thruster_mode = (int) v;
			}
			printf("set_thruster_mode : completed\n");
		}
	} else if (strcmp(cmd, "set_light_strength") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v;
			int num = sscanf(param, "%f", &v);

			if (num == 1) {
				lg_light_strength = v;
			}
			printf("set_light_strength : completed\n");
		}
	} else if (strcmp(cmd, "set_lowlevel_control") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_lowlevel_control = (value != 0);
			printf("set_lowlevel_control : completed\n");
		}
	} else if (strcmp(cmd, "set_emergency_mode") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_emergency_mode = (value != 0);

			printf("set_emergency_mode : completed\n");
		}
	} else if (strcmp(cmd, "set_pid_enabled") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_pid_enabled = (value != 0);
			lg_thrust = 0;
			lg_rudder = 0;
			memset(lg_pid_value, 0, sizeof(lg_pid_value));
			memset(lg_delta_pid_target, 0, sizeof(lg_delta_pid_target));

			printf("set_pid_enabled : completed\n");
		}
	} else if (strcmp(cmd, "set_heading_lock") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_heading_lock = (value != 0);

			printf("set_heading_lock : completed\n");
		}
	} else if (strcmp(cmd, "set_light_value") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int id = 0;
			float value = 0;
			sscanf(param, "%d=%f", &id, &value);
			if (id < LIGHT_NUM) {
				lg_light_value[id] = value;
			}
			sscanf(param, "%f", &value);
			printf("set_light_value : completed\n");
		}
	} else if (strcmp(cmd, "set_motor_value") == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int id = 0;
			float value = 0;
			sscanf(param, "%d=%f", &id, &value);
			if (id < MOTOR_NUM) {
				lg_motor_value[id] = value;
			}
			printf("set_motor_value : completed\n");
		}
	} else {
		printf(":unknown command : %s\n", cmd);
	}
	return 0;
}

// static void init_options(void *user_data, json_t *_options) {
// 	PLUGIN_T *plugin = (PLUGIN_T*) user_data;
// 	json_t *options = json_object_get(_options, PLUGIN_NAME);

// 	{ //pid_gain
// 		json_t *ary1 = json_object_get(options, "pid_gain");
// 		if (json_is_array(ary1)) {
// 			int size1 = json_array_size(ary1);
// 			for (int i1 = 0; i1 < MIN(size1, PID_NUM); i1++) {
// 				json_t *ary2 = json_array_get(ary1, i1);
// 				if (json_is_array(ary2)) {
// 					int size2 = json_array_size(ary2);
// 					for (int i2 = 0; i2 < MIN(size2, 3); i2++) {
// 						lg_pid_gain[i1][i2] = json_number_value(json_array_get(ary2, i2));
// 					}
// 				}
// 			}
// 		}
// 	}

// 	for (int i = 0; i < LIGHT_NUM; i++) {
// 		char buff[256];
// 		sprintf(buff, "light%d_id", i);
// 		int id = (int) json_number_value(json_object_get(options, buff));
// 		if (id != 0) {
// 			lg_light_id[i] = id;
// 		}
// 	}

// 	for (int i = 0; i < MOTOR_NUM; i++) {
// 		char buff[256];
// 		int value;
// 		sprintf(buff, "motor%d_id", i);
// 		value = (int) json_number_value(json_object_get(options, buff));
// 		if (value != 0) {
// 			lg_motor_id[i] = value;
// 		}
// 		sprintf(buff, "motor%d_dir", i);
// 		value = (int) json_number_value(json_object_get(options, buff));
// 		if (value != 0) {
// 			lg_motor_dir[i] = value;
// 		}
// 	}
// 	{
// 		float value;
// 		value = json_number_value(json_object_get(options, "motor_center"));
// 		if (value != 0) {
// 			lg_motor_center = value;
// 		}
// 		value = json_number_value(json_object_get(options, "motor_margin"));
// 		if (value != 0) {
// 			lg_motor_margin = value;
// 		}
// 		value = json_number_value(json_object_get(options, "motor_range"));
// 		if (value != 0) {
// 			lg_motor_range = value;
// 		}
// 	}
// 	lg_thruster_mode = (int) json_number_value(json_object_get(options, "thruster_mode"));

// 	init_pwm(); //need motor ids
// }

// static void save_options(void *user_data, json_t *_options) {
// 	json_t *options = json_object();
// 	json_object_set_new(_options, PLUGIN_NAME, options);

// 	{ //pid_gain
// 		json_t *ary1 = json_array();
// 		for (int i1 = 0; i1 < PID_NUM; i1++) {
// 			json_t *ary2 = json_array();
// 			for (int i2 = 0; i2 < 3; i2++) {
// 				json_array_append_new(ary2, json_real(lg_pid_gain[i1][i2]));
// 			}
// 			json_array_append_new(ary1, ary2);
// 		}
// 		json_object_set_new(options, "pid_gain", ary1);
// 	}

// 	for (int i = 0; i < LIGHT_NUM; i++) {
// 		char buff[256];
// 		sprintf(buff, "light%d_id", i);
// 		json_object_set_new(options, buff, json_real(lg_light_id[i]));
// 	}

// 	for (int i = 0; i < MOTOR_NUM; i++) {
// 		char buff[256];
// 		sprintf(buff, "motor%d_id", i);
// 		json_object_set_new(options, buff, json_real(lg_motor_id[i]));
// 		sprintf(buff, "motor%d_dir", i);
// 		json_object_set_new(options, buff, json_real(lg_motor_dir[i]));
// 	}

// 	{
// 		json_object_set_new(options, "motor_center", json_real(lg_motor_center));
// 		json_object_set_new(options, "motor_margin", json_real(lg_motor_margin));
// 		json_object_set_new(options, "motor_range", json_real(lg_motor_range));
// 	}
// 	json_object_set_new(options, "thruster_mode", json_real(lg_thruster_mode));
// }