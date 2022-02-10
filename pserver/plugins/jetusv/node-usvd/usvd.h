#pragma once


#ifdef __cplusplus

extern "C" {

#endif

typedef void (*USVD_PWM_CALLBACK)(int ch, float v_us, void *arg);

void usvd_init(const char *config_json);
void usvd_set_pwm_callback(USVD_PWM_CALLBACK callback, void *arg);
void usvd_poll();

#ifdef __cplusplus

}

#endif