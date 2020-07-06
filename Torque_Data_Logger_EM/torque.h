#include "mbed.h"
#include "QEI.h"
#include "PinDetect.h"

typedef enum
{
    RUN,
    STOP

} motorState_t;

class TORQUE
{
    public:

    TORQUE(PinName motorFlush);

    float getTorque(void);
    float getCurrent(void);
    float getVoltage(void);
    float getFlushLenght(void);


    uint32_t getFlushCount(void);
    uint32_t getUsesCount(void);
    uint8_t getRpm(void);


    void writeEEPROMData(void);
    void clearEEPROMData(void);
    void readEEPROMData(void);


    private:

    PinName _motorFlush;

    PinDetect* _pdMotorFlush;

    int32_t _rotationCount;
    int32_t _previousRotationCount;

    uint32_t _flushCount;
    uint32_t _UsesCount;
    uint32_t _Services;

    float _Voltage;
    float _Current;
    float _Rpm;


    motorState_t _motorCurrentState;
   
    void _flushPressed (void);
    void _flushReleased (void);
    void _flushHeld (void);

};