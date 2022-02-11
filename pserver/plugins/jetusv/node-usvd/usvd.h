#pragma once


#ifdef __cplusplus

extern "C" {

#endif

typedef void (*USVD_PWM_CALLBACK)(int ch, float v_us, void *arg);

void usvd_init(const char *config_json, USVD_PWM_CALLBACK callback, void *arg);
void usvd_poll(float t_s, float north);
int usvd_command(const char *cmd);

#ifdef __cplusplus

}

#endif