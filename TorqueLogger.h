#include "mbed.h"
#include "PinDetect.h"



typedef enum
{
    RUN,
    STOP

} flushState_t;

class TorqueLogger
{
    public:

    TorqueLogger(PinName FlushButton);

    float getTorque(void);
    float getCurrent(void);
    float getVoltage(void);
    float getFlushLenght(void);
    float getRpm(void);
    float getPower(void);


    uint32_t getFlushCount(void);
    uint32_t getUsesCount(void);
    uint32_t getServiceCount(void);
    


    private:

    PinName _motorFlush;

    PinDetect* _pdMotorFlush;

    int32_t _rotationCount;
    int32_t _previousRotationCount;

    uint32_t _FlushCount;
    uint32_t _UsesCount;
    uint32_t _ServiceCount;

    uint8_t _Torque;

    float _Voltage;
    double _Current;
    float _Rpm;
    float _Power;


    flushState_t _FlushState;
   
    void _FlushPressed (void);
    void _FlushReleased (void);
    void _FlushHeld (void);

};