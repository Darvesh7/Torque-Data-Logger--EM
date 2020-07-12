#include "mbed.h"
#include "QEI.h"



typedef enum
{
    MFORWARD,
    MBACKWARD,
    MSTOP,
    MBRAKE
} motorState_t;

class Motor
{
     public:
    // CONSTRUCTOR
    // User-defined pin selection.
    Motor(PinName INA, 
    PinName INB,
    PinName PWM,
    PinName SEL0, 
    PinName CS);

    void init(void);

    void forward(void);
    void backward(void);
    void stop(void);
    void brake(void);


    void setMotorSpeed(void);
    void setMotorBrake(void);
    void setMotorSpeed(int);
    void getMotorSpeed(void);
    
    float getCurrent(void); 

    float pwm_value; 
    float pwm_value_brake;
    float pwm_i; //pwm increment
    float pwm_d; //pwm decrement


    motorState_t _MState;



    private:

    PinName _INA;
    PinName _INB;
    PinName _PWM;
    PinName _SEL0;
    PinName _CS;


    DigitalOut* _motorINA;
    DigitalOut* _motorINB;
    PwmOut* _motorPWM;
    DigitalOut* _motorSEL0;
    AnalogIn* _motorCS;

    float _motorCurrent;


};

