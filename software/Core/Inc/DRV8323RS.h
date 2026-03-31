#ifndef DRV8323RS_H
#define DRV8323RS_H

#include <stdint.h>
#define PWM_MODE_6              0b00
#define PWM_MODE_3              0b01
#define PWM_MODE_1              0b10
#define PWM_MODE_INDEPENDENET   0b11

typedef enum {
	DRV_OK          = 0,
	DRV_ERROR       = 1,
	DRV_HAL_ERROR   = 2,
	DRV_HAL_TIMEOUT = 3,
} DRV8323_ErrorTypeDef;

DRV8323_ErrorTypeDef disable_charge_pump(uint8_t);

DRV8323_ErrorTypeDef disable_gate_drive_fault(uint8_t);

DRV8323_ErrorTypeDef set_pwm_mode(uint8_t);

#endif
