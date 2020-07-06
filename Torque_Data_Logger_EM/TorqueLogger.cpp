#include "mbed.h"
#include "TorqueLogger.h"
#include "QEI.h"
#include "PinDetect.h"


TorqueLogger::TorqueLogger(PinName motorFlush): _pdMotorFlush(new PinDetect (motorFlush))

{
    _motorCurrentState = STOP;
    _rotationCount = 0;
    _previousRotationCount = 0;
    _flushCount = 0;
    _UsesCount = 0;
    _ServiceCount = 0;


    _pdMotorFlush->mode(PullUp);
    _pdMotorFlush->setAssertValue(0);
    _pdMotorFlush->attach_asserted(this, &TorqueLogger::_FlushPressed);
    _pdMotorFlush->attach_deasserted(this, &TorqueLogger::_FlushReleased);
    _pdMotorFlush->attach_asserted_held(this, &TorqueLogger::_FlushHeld); //if button is pressed for 2 seconds and released

    _pdMotorFlush->setSamplesTillHeld( 100 );
    _pdMotorFlush->setSampleFrequency();



}

void TorqueLogger::readEEPROMData(void)
{

}

void TorqueLogger::writeEEPROMData(void)
{

}


float TorqueLogger::getVoltage(void)
{

}

float TorqueLogger::getCurrent(void)
{


}

float TorqueLogger::getFlushLenght(void)
{

}


float TorqueLogger::getTorque(void)
{


}


uint32_t TorqueLogger::getFlushCount(void)
{

}

uint32_t TorqueLogger::getUsesCount(void)
{


}

uint32_t TorqueLogger::getServiceCount(void)
{

}
