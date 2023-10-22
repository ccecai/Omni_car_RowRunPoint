//
// Created by Yanzhenbo on 2023/4/1.
//

#ifndef CHASSIS_ER_DGM_H
#define CHASSIS_ER_DGM_H
#include "main.h"

#define DGM_FRONT_ID 0x001
#define DGM_LEFT_ID 0X002
#define DGM_RIGHT_ID 0X003

typedef enum eCanCmd{
    CAN_CMD_MOTOR_DISABLE = 0,
    CAN_CMD_MOTOR_ENABLE,

    CAN_CMD_SET_TORQUE,
    CAN_CMD_SET_VELOCITY,
    CAN_CMD_SET_POSITION,
    CAN_CMD_SYNC,

    CAN_CMD_CALIB_START,
    CAN_CMD_CALIB_REPORT,
    CAN_CMD_CALIB_ABORT,

    CAN_CMD_ANTICOGGING_START,
    CAN_CMD_ANTICOGGING_REPORT,
    CAN_CMD_ANTICOGGING_ABORT,

    CAN_CMD_SET_HOME,
    CAN_CMD_ERROR_RESET,
    CAN_CMD_GET_STATUSWORD,
    CAN_CMD_STATUSWORD_REPORT,

    CAN_CMD_GET_TORQUE,
    CAN_CMD_GET_VELOCITY,
    CAN_CMD_GET_POSITION,
    CAN_CMD_GET_I_Q,
    CAN_CMD_GET_VBUS,
    CAN_CMD_GET_IBUS,
    CAN_CMD_GET_POWER,

    CAN_CMD_SET_CONFIG,
    CAN_CMD_GET_CONFIG,
    CAN_CMD_SAVE_ALL_CONFIG,
    CAN_CMD_RESET_ALL_CONFIG,

    CAN_CMD_GET_FW_VERSION = 50,
    CAN_CMD_WRITE_APP_BACK_START,
    CAN_CMD_WRITE_APP_BACK,
    CAN_CMD_CHECK_APP_BACK,
    CAN_CMD_DFU_START,

    CAN_CMD_HEARTBEAT = 63,
}tCanCmd;

typedef struct {
    uint32_t id:24;
    uint32_t dlc:8;
    uint8_t  data[8];
} CanFrame;

typedef union
{
    int8_t		data_int8[8];
    uint8_t 	data_uint8[8];
    int16_t 	data_int16[4];
    uint16_t	data_uint16[4];
    int32_t		data_int32[2];
    uint32_t	data_uint32[2];
    float 		data_float[2];
}Data_Format_Trans;

typedef struct {
    int poles;
    int control_mode;
    float vel_gain;
    float vel_integrator;
    float vel_limit;
    float current_limit;
    float inertia;
    float velocity;
    float current;
    int step;
} DgmConfigInfo;


void DGM_SET_TARGET_VELOCITY(CAN_HandleTypeDef *_hcan, uint16_t dgm_id, float target_speed);
void DGM_CAN_CMD_MOTOR_ENABLE(CAN_HandleTypeDef *_hcan, uint16_t dgm_id);

#endif //CHASSIS_ER_DGM_H
