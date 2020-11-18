// MIT License

// Copyright (c) 2020 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _DCMOTOR_H_
#define _DCMOTOR_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include "stdbool.h"

#include "stm_err.h"
#include "driver/timer.h"

typedef struct dcmotor *dcmotor_handle_t;

typedef struct {
	timer_num_t					a_timer_num;			/*!< Pin A timer num */
	timer_chnl_t				a_timer_chnl;			/*!< Pin A timer channel */
	timer_pins_pack_t 			a_timer_pins_pack;		/*!< Pin A timer pins pack */
	timer_num_t					b_timer_num;			/*!< Pin B timer num */
	timer_chnl_t				b_timer_chnl;			/*!< Pin B timer channel */
	timer_pins_pack_t 			b_timer_pins_pack;		/*!< Pin B timer pins pack */
} dcmotor_hardware_info_t;

typedef struct {
	dcmotor_hardware_info_t 	hw_info;				/*!< Hardware information */
	bool 						dir;					/*!< Direction */
	uint32_t 					freq;					/*!< PWM frequency */
	uint8_t 					duty;					/*!< PWM duty cycle */
} dcmotor_cfg_t;

/*
 * @brief   Initialize DC motor.
 * @param   config Struct pointer.
 * @return
 *      - DC motor handle structure: Success.
 *      - 0: Fail
 */
dcmotor_handle_t dcmotor_init(dcmotor_cfg_t *config);

/*
 * @brief   Set DC motor direction.
 * @param   handle Handle structure.
 * @param   dir Direction.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t dcmotor_set_dir(dcmotor_handle_t handle, bool dir);

/*
 * @brief   Toggle DC motor direction.
 * @param   handle Handle structure.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t dcmotor_toggle_dir(dcmotor_handle_t handle);

/*
 * @brief   Set DC motor PWM frequency.
 * @param   handle Handle structure.
 * @param 	freq PWM frequency in Hz.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t dcmotor_set_pwm_freq(dcmotor_handle_t handle, uint32_t freq);

/*
 * @brief   Set DC motor PWM duty.
 * @param   handle Handle structure.
 * @param 	duty PWM duty in %.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t dcmotor_set_pwm_duty(dcmotor_handle_t handle, uint8_t duty);

/*
 * @brief   Start DC motor.
 * @param   handle Handle structure.
 * @return
 *      - STM_OK:       Success.
 *      - Others:       Fail.
 */
stm_err_t dcmotor_start(dcmotor_handle_t handle);

#ifdef __cplusplus 
}
#endif

#endif /* _DCMOTOR_H_ */