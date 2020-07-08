#include "mbed.h"

typedef enum 
{ 
    S_RUN, 
    S_SER,
    S_SET 
} sysMode_t;

typedef struct
{
    sysMode_t sysMode;
}sysState_struct_t;

typedef enum 
{ 
    MA_Forward, 
    MA_Backward,
    MA_Brake,
    MA_Stop
} MA_t;

MA_t gMotorAction = MA_Stop;

sysState_struct_t sysState_struct;