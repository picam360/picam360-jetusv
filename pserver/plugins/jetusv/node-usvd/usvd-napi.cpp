#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <string>
#include <node_api.h>
#include "usvd.h"

using namespace std;

#define NAPI_CALL(env, call)                                               \
do {                                                                       \
  napi_status status = call;                                               \
  if (status != napi_ok) {                                                 \
	printf("status %d : ", status);                                        \
    napi_fatal_error(#call, NAPI_AUTO_LENGTH, "failed", NAPI_AUTO_LENGTH); \
  }                                                                        \
} while (0)

typedef struct CALLBACK_DATA{
	int ch;
	float v_us;
}CALLBACK_DATA;
static void usvd_pwm_callback(int ch, float v_us, void *arg) {
	napi_threadsafe_function callback = (napi_threadsafe_function)arg;

	CALLBACK_DATA *data = new CALLBACK_DATA();
	data->ch = ch;
	data->v_us = v_us;

	NAPI_CALL(env, napi_acquire_threadsafe_function(callback));
	NAPI_CALL(env,
			napi_call_threadsafe_function(callback, (void* )data,
					napi_tsfn_blocking));

	return;
}

extern "C" {
static void js_usvd_pwm_callback(napi_env env, napi_value js_callback, void *_ctx,
		void *_data) {
	CALLBACK_DATA *data = (CALLBACK_DATA*) _data;

	napi_value nv_ch;
	NAPI_CALL(env, napi_create_int64(env, data->ch, &nv_ch));

	napi_value nv_v_us;
	NAPI_CALL(env, napi_create_double(env, data->v_us, &nv_v_us));

	napi_value argv[] = { nv_ch, nv_v_us };
	napi_value undefined;
	napi_value ret;
	NAPI_CALL(env, napi_get_undefined(env, &undefined));
	napi_status status = napi_call_function(env, undefined, js_callback, 2, argv, &ret);
	if (status != napi_ok) {
		printf("something wrong %d : ", status);
	}

	delete data;
}
}

static napi_value napi_usvd_init(napi_env env,
		napi_callback_info info) {
	size_t argc = 2;
	napi_value argv[2];
	NAPI_CALL(env, napi_get_cb_info(env, info, &argc, argv, NULL, NULL));
	if (argc != 2) {
		return NULL;
	}

	napi_valuetype argument_type;
	NAPI_CALL(env, napi_typeof(env, argv[0], &argument_type));
	if (argument_type != napi_string) {
		return NULL;
	}

	NAPI_CALL(env, napi_typeof(env, argv[1], &argument_type));
	if (argument_type != napi_function) {
		return NULL;
	}

	char config_json[1024] = { };
	size_t copied;

	NAPI_CALL(env,
			napi_get_value_string_utf8(env, argv[0], config_json, sizeof(config_json),
					&copied));

	napi_value resource_name;
	NAPI_CALL(env,
			napi_create_string_utf8(env, "usvd", NAPI_AUTO_LENGTH,
					&resource_name));

	napi_threadsafe_function callback = NULL;
	NAPI_CALL(env, //
			napi_create_threadsafe_function(env, argv[1],//func
			NULL,//async_resource
			resource_name,//async_resource_name
			0,//max_queue_size
			2,//initial_thread_count
			NULL,//thread_finalize_data
			NULL,//thread_finalize_cb
			(void*)0x0,//context
			js_usvd_pwm_callback,//call_js_cb
			&callback//result
			));


	usvd_init(config_json, usvd_pwm_callback, callback);

	return NULL;
}

static napi_value napi_usvd_poll(napi_env env,
		napi_callback_info info) {
	size_t argc = 2;
	napi_value argv[2];
	NAPI_CALL(env, napi_get_cb_info(env, info, &argc, argv, NULL, NULL));
	if (argc != 2) {
		return NULL;
	}

	napi_valuetype argument_type;
	NAPI_CALL(env, napi_typeof(env, argv[0], &argument_type));
	if (argument_type != napi_number) {
		return NULL;
	}
	NAPI_CALL(env, napi_typeof(env, argv[1], &argument_type));
	if (argument_type != napi_number) {
		return NULL;
	}

	double t_s;
	NAPI_CALL(env, napi_get_value_double(env, argv[0], &t_s));
	
	double north;
	NAPI_CALL(env, napi_get_value_double(env, argv[1], &north));

	usvd_poll(t_s, north);

	return NULL;
}

static napi_value napi_usvd_command(napi_env env,
		napi_callback_info info) {
	size_t argc = 1;
	napi_value argv[1];
	NAPI_CALL(env, napi_get_cb_info(env, info, &argc, argv, NULL, NULL));
	if (argc != 1) {
		return NULL;
	}

	napi_valuetype argument_type;
	NAPI_CALL(env, napi_typeof(env, argv[0], &argument_type));
	if (argument_type != napi_string) {
		return NULL;
	}

	char cmd[1024] = { };
	size_t copied;

	NAPI_CALL(env,
			napi_get_value_string_utf8(env, argv[0], cmd, sizeof(cmd),
					&copied));

	int _ret = usvd_command(cmd);

	napi_value ret;
	napi_create_int64(env, (uint64_t) _ret, &ret);
	return ret;
}

#define DECLARE_NAPI_METHOD(name, func)                                        \
  { name, 0, func, 0, 0, 0, napi_default, 0 }

static napi_value Init(napi_env env, napi_value exports) {
	napi_status status;
	napi_property_descriptor desc[] = {
	DECLARE_NAPI_METHOD("usvd_init",
			napi_usvd_init),
	DECLARE_NAPI_METHOD("usvd_poll",
			napi_usvd_poll),
	DECLARE_NAPI_METHOD("usvd_command",
			napi_usvd_command), };
	status = napi_define_properties(env, exports,
			sizeof(desc) / sizeof(desc[0]), desc);
	assert(status == napi_ok);
	return exports;
}

NAPI_MODULE(pstcore, Init)
