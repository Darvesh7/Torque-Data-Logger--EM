#include "mbed.h"
#include "TorqueLogger.h"
#include "QEI.h"
#include "PinDetect.h"
#include "motor.h"
#include "ACS712.h"

AnalogIn voltagePin(PA_0);
ACS712 CurrentSensor(PA_1,1,30);

extern ACS712 CurrentSensor;
extern Motor Motor;
extern QEI encoder;

Timer rpmTimer;



unsigned long start = 0;
double ppr = 16; 
uint8_t upDatesPerSec = 2;
const int fin = 1000 / upDatesPerSec;
const float constant = 60.0 * upDatesPerSec / (ppr*2);
float rpm;



TorqueLogger::TorqueLogger(PinName FlushButton): 

_pdMotorFlush(new PinDetect (FlushButton))

{
    _FlushState = STOP;
    _rotationCount = 0;
    _previousRotationCount = 0;
    _FlushCount = 0;
    _UsesCount = 0;
    _ServiceCount = 0;
    _Torque = 0.0;
    _Voltage = 0.0;
    _Current = 0.0;
    _Power = 0.0;



}



float TorqueLogger::getVoltage(void)
{
    _Voltage = voltagePin.read_u16()*(16.70/65535.00);
    //printf("Voltage %2.2f\n\r",_Voltage); 
    return _Voltage;
}

float TorqueLogger::getCurrent(void)
{
    _Current = float(CurrentSensor);
    //printf("Current %2.2f\n\r",_Current); 
    return _Current;
}


float TorqueLogger::getPower(void)
{

    _Power = voltagePin.read_u16()*(16.70/65535.00) * float(CurrentSensor);
    //printf("Power %2.2f\n\r",(_Power)); 
    return (_Power);
}


float TorqueLogger::getRpm(void)
{

    while(1)
    {
        rpmTimer.start();
        if (rpmTimer.read_ms() - start > fin) 
        {
        start = rpmTimer.read_ms();
        rpm = abs((encoder.getPulses() * (constant)/149.25));
    
        //printf("%2.2f\n", (rpm));
        encoder.reset();
        rpmTimer.reset();
        }
    }

return _Rpm;
    
}




float TorqueLogger::getTorque(void)
{
   
    return _Torque;
}



uint32_t TorqueLogger::getFlushCount(void)
{
    return _FlushCount;
}

uint32_t TorqueLogger::getUsesCount(void)
{
    return _UsesCount;
}

uint32_t TorqueLogger::getServiceCount(void)
{
    return _ServiceCount;
}



void TorqueLogger::_FlushPressed(void)
{
   Motor.forward();
   getRpm();

}

void TorqueLogger::_FlushReleased(void)
{
    Motor.brake();
    
}

void TorqueLogger::_FlushHeld(void)
{


}




