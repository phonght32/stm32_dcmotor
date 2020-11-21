#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "include/dcmotor.h"

#define DCMOTOR_INIT_ERR_STR			"dcmotor init error"
#define DCMOTOR_START_ERR_STR			"dcmotor start error"
#define DCMOTOR_STOP_ERR_STR			"dcmotor stop error"
#define DCMOTOR_SET_PWM_FREQ_ERR_STR	"dcmotor set pwm frequency error"
#define DCMOTOR_SET_PWM_DUTY_ERR_STR	"dcmotor set pwm duty cycle error"
#define DCMOTOR_SET_DIR_ERR_STR			"dcmotor set direction error"
#define DCMOTOR_TOGGLE_DIR_ERR_STR		"dcmotor toggle direction error"

#define mutex_lock(x)               while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)             xSemaphoreGive(x)
#define mutex_create()              xSemaphoreCreateMutex()
#define mutex_destroy(x)            vQueueDelete(x)

static const char* TAG = "DCMOTOR";
#define DCMOTOR_CHECK(a, str, action)  if(!(a)) {                                           \
        STM_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);       		\
        action;                                                                             \
        }

typedef struct dcmotor {
	dcmotor_hw_info_t 			hw_info;
	bool 						dir;
	uint32_t 					freq;
	uint8_t 					duty;
	SemaphoreHandle_t			lock;
} dcmotor_t;

void _dcmotor_cleanup(dcmotor_handle_t handle)
{
	free(handle);
}

dcmotor_handle_t dcmotor_init(dcmotor_cfg_t *config)
{
	/* Check input conditions */
	DCMOTOR_CHECK(config, DCMOTOR_INIT_ERR_STR, return NULL);
	DCMOTOR_CHECK(config->hw_info.a_timer_num < TIMER_NUM_MAX, DCMOTOR_INIT_ERR_STR, return NULL);
	DCMOTOR_CHECK(config->hw_info.a_timer_chnl < TIMER_CHNL_MAX, DCMOTOR_INIT_ERR_STR, return NULL);
	DCMOTOR_CHECK(config->hw_info.a_timer_pins_pack < TIMER_PINS_PACK_MAX, DCMOTOR_INIT_ERR_STR, return NULL);
	DCMOTOR_CHECK(config->hw_info.b_timer_num < TIMER_NUM_MAX, DCMOTOR_INIT_ERR_STR, return NULL);
	DCMOTOR_CHECK(config->hw_info.b_timer_chnl < TIMER_CHNL_MAX, DCMOTOR_INIT_ERR_STR, return NULL);
	DCMOTOR_CHECK(config->hw_info.b_timer_pins_pack < TIMER_PINS_PACK_MAX, DCMOTOR_INIT_ERR_STR, return NULL);

	/* Allocate memory for handle structure */
	dcmotor_handle_t handle = calloc(1, sizeof(dcmotor_t));
	DCMOTOR_CHECK(handle, DCMOTOR_INIT_ERR_STR, return NULL);

	/* Configure pin a */
	pwm_cfg_t a_cfg;
	a_cfg.timer_num = config->hw_info.a_timer_num;
	a_cfg.timer_pins_pack = config->hw_info.a_timer_pins_pack;
	a_cfg.timer_chnl = config->hw_info.a_timer_chnl;
	DCMOTOR_CHECK(!pwm_config(&a_cfg), DCMOTOR_INIT_ERR_STR, {_dcmotor_cleanup(handle); return NULL;});

	/* Configure pin b */
	pwm_cfg_t b_cfg;
	b_cfg.timer_num = config->hw_info.b_timer_num;
	b_cfg.timer_pins_pack = config->hw_info.b_timer_pins_pack;
	b_cfg.timer_chnl = config->hw_info.b_timer_chnl;
	DCMOTOR_CHECK(!pwm_config(&b_cfg), DCMOTOR_INIT_ERR_STR, {_dcmotor_cleanup(handle); return NULL;});

	/* Set direction */
	if (config->dir) {
		DCMOTOR_CHECK(!pwm_set_params(config->hw_info.a_timer_num, config->hw_info.a_timer_chnl, config->freq, config->duty), DCMOTOR_INIT_ERR_STR, {_dcmotor_cleanup(handle); return NULL;});
		DCMOTOR_CHECK(!pwm_set_params(config->hw_info.b_timer_num, config->hw_info.b_timer_chnl, 0, 0), DCMOTOR_INIT_ERR_STR, {_dcmotor_cleanup(handle); return NULL;});
	} else {
		DCMOTOR_CHECK(!pwm_set_params(config->hw_info.a_timer_num, config->hw_info.a_timer_chnl, 0, 0), DCMOTOR_INIT_ERR_STR, {_dcmotor_cleanup(handle); return NULL;});
		DCMOTOR_CHECK(!pwm_set_params(config->hw_info.b_timer_num, config->hw_info.b_timer_chnl, config->freq, config->duty), DCMOTOR_INIT_ERR_STR, {_dcmotor_cleanup(handle); return NULL;});
	}

	/* Update handle structure */
	handle->hw_info.a_timer_num = config->hw_info.a_timer_num;
	handle->hw_info.a_timer_chnl = config->hw_info.a_timer_chnl;
	handle->hw_info.a_timer_pins_pack = config->hw_info.a_timer_pins_pack;
	handle->hw_info.b_timer_num = config->hw_info.b_timer_num;
	handle->hw_info.b_timer_chnl = config->hw_info.b_timer_chnl;
	handle->hw_info.b_timer_pins_pack = config->hw_info.b_timer_pins_pack;
	handle->dir = config->dir;
	handle->freq = config->freq;
	handle->duty = config->duty;
	handle->lock = mutex_create();

	return handle;
}

stm_err_t dcmotor_set_dir(dcmotor_handle_t handle, bool dir)
{
	DCMOTOR_CHECK(handle, DCMOTOR_SET_DIR_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	int ret;
	if (dir) {
		ret = pwm_set_params(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl, handle->freq, handle->duty);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_DIR_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}

		ret = pwm_set_params(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl, 0, 0);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_DIR_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	} else {
		ret = pwm_set_params(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl, 0, 0);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_DIR_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}

		ret = pwm_set_params(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl, handle->freq, handle->duty);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_DIR_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	}

	handle->dir = dir;
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t dcmotor_toggle_dir(dcmotor_handle_t handle)
{
	DCMOTOR_CHECK(handle, DCMOTOR_TOGGLE_DIR_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	int ret;
	bool dir = !handle->dir;
	if (dir) {
		ret = pwm_set_params(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl, handle->freq, handle->duty);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_TOGGLE_DIR_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}

		ret = pwm_set_params(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl, 0, 0);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_TOGGLE_DIR_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	} else {
		ret = pwm_set_params(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl, 0, 0);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_TOGGLE_DIR_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}

		ret = pwm_set_params(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl, handle->freq, handle->duty);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_TOGGLE_DIR_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	}

	handle->dir = dir;
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t dcmotor_set_pwm_freq(dcmotor_handle_t handle, uint32_t freq)
{
	DCMOTOR_CHECK(handle, DCMOTOR_SET_PWM_FREQ_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	int ret;
	if (handle->dir) {
		ret = pwm_set_params(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl, freq, handle->duty);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_PWM_FREQ_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}

		ret = pwm_set_params(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl, 0, 0);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_PWM_FREQ_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	} else {
		ret = pwm_set_params(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl, 0, 0);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_PWM_FREQ_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}

		ret = pwm_set_params(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl, freq, handle->duty);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_PWM_FREQ_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	}

	handle->freq = freq;
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t dcmotor_set_pwm_duty(dcmotor_handle_t handle, uint8_t duty)
{
	DCMOTOR_CHECK(handle, DCMOTOR_SET_PWM_DUTY_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	int ret;
	if (handle->dir) {
		ret = pwm_set_params(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl, handle->freq, duty);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_PWM_DUTY_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}

		ret = pwm_set_params(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl, 0, 0);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_PWM_DUTY_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	} else {
		ret = pwm_set_params(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl, 0, 0);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_PWM_DUTY_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}

		ret = pwm_set_params(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl, handle->freq, duty);
		if (ret) {
			STM_LOGE(TAG, DCMOTOR_SET_PWM_DUTY_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	}

	handle->duty = duty;
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t dcmotor_start(dcmotor_handle_t handle)
{
	DCMOTOR_CHECK(handle, DCMOTOR_START_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	int ret;
	ret = pwm_start(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl);
	if (ret) {
		STM_LOGE(TAG, DCMOTOR_START_ERR_STR);
		mutex_unlock(handle->lock);
		return STM_FAIL;
	}
	ret = pwm_start(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl);
	if (ret) {
		STM_LOGE(TAG, DCMOTOR_START_ERR_STR);
		mutex_unlock(handle->lock);
		return STM_FAIL;
	}

	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t dcmotor_stop(dcmotor_handle_t handle)
{
	DCMOTOR_CHECK(handle, DCMOTOR_STOP_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	int ret;
	ret = pwm_stop(handle->hw_info.a_timer_num, handle->hw_info.a_timer_chnl);
	if (ret) {
		STM_LOGE(TAG, DCMOTOR_STOP_ERR_STR);
		mutex_unlock(handle->lock);
		return STM_FAIL;
	}
	ret = pwm_stop(handle->hw_info.b_timer_num, handle->hw_info.b_timer_chnl);
	if (ret) {
		STM_LOGE(TAG, DCMOTOR_STOP_ERR_STR);
		mutex_unlock(handle->lock);
		return STM_FAIL;
	}

	mutex_unlock(handle->lock);

	return STM_OK;
}

void dcmotor_destroy(dcmotor_handle_t handle)
{
	free(handle);
}