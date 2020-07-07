#include "mbed.h"
#include "motor.h"
#include "QEI.h"
#include "PinDetect.h"
#include "ACS712.h"


#define ENCPPR 2338

#define MIN_FILM 1000 
#define P_THRES 250
#define ONE_FLUSH 10000 

float FILM_LEFT = 12000.0; 
float FILM_LENGTH = 12000.0; 

bool update_film_value = false;
bool update_sensor_data = false;
float FilmLeft = 0.0;
int pulses = 0;
int cummulative_pulses = 0;

float Voltage = 0.0;
float Current = 0.0;
float Power = 0.0;
float Torque = 0.0;


PinDetect Flush_button (PC_13);
Motor motor(PA_10,PB_3,PB_5,PB_4,PA_4);
QEI encoder(PC_1,PC_0,NC,ENCPPR);
AnalogIn voltagePin(PA_0);
ACS712 CurrentSensor(PA_1,1,30);

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

Semaphore motorSema(1);


Thread t1;
Thread t2;
Thread VoltageThread, CurrentThread, PowerThread, RpmThread, TorqueThread;

EventFlags stateChanged, dataLog;
Timeout flush_end;
Timer rpmTimer;

uint32_t flags_read = 0;

unsigned long start = 0;
double ppr = 16; 
uint8_t upDatesPerSec = 2;
const int fin = 1000 / upDatesPerSec;
const float constant = 60.0 * upDatesPerSec / (ppr*2);
float rpm;

void SystemStates_thread(void const *name);
void motorDrive_thread(void const *name); 



void start_flush(void);
void end_flush(void);
void setup_flush_button(void);
void getVoltage(void);
void getCurrent(void);
void getPower(void);
void getTorque(void);
void getRpm(void);


void setup()
{
    setup_flush_button();

    t1.start(callback(motorDrive_thread, (void *)"MotorDriveThread"));
    t2.start(callback(SystemStates_thread, (void *)"SystemStateThread"));


    sysState_struct.sysMode = S_RUN;

    VoltageThread.start(getVoltage);
    CurrentThread.start(getCurrent);
    PowerThread.start(getPower);
    RpmThread.start(getRpm);
    TorqueThread.start(getTorque);

}



int main()

{
    setup();

    while (true) 
    {
    sleep();
    
        if (update_film_value) 
        {

        printf("FILM_LENGTH %f\n", FILM_LENGTH);
        printf("Power %2.2f\n\r",(Voltage)); 
        pulses = 0;
        encoder.reset();
        update_film_value = false;
        }

        if (update_sensor_data) 
        {
        printf("Voltage %f\n", Voltage);
        
        //printf("Power %2.2f\n\r",Current); 
        update_film_value = false;
        }

    }
}



void getVoltage(void)
{
    flags_read = dataLog.wait_any(0x01);
    while(true)
    {

    Voltage = voltagePin.read_u16()*(16.70/65535.00);
    //printf("Voltage %2.2f\n\r",Voltage); 
    }
    ThisThread::sleep_for(10);
    
}

void getCurrent(void)
{

    while(true)
    {
    
    Current = float(CurrentSensor);
    //printf("Current %2.2f\n\r",Current); 
    }
    //ThisThread::sleep_for(1000);
   
}


void getPower(void)
{
    while(true)
    {   
    flags_read = dataLog.wait_any(0x01);
    Power = voltagePin.read_u16()*(16.70/65535.00) * float(CurrentSensor);
    //printf("Power %2.2f\n\r",(Power)); 
    }
    ThisThread::sleep_for(1000);
}

void getRpm(void)
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
    ThisThread::sleep_for(1000);

}

void getTorque(void)
{
    while(1)
    {
    Torque = Power/(2*(2*3.14)*rpm);
    }
    ThisThread::sleep_for(1000);
}



void start_flush(void)
{
    
    
    if(sysState_struct.sysMode == S_RUN)
    {
        if(motor._MState == MSTOP)
        {
        gMotorAction = MA_Forward;
        motorSema.release();
        flush_end.attach(&end_flush, 4.0); //timeout - duration of flush
        }
    }
 
}

void end_flush(void)
{
    if(motor._MState == MFORWARD)
    {
        dataLog.set(0x01);
        gMotorAction = MA_Brake;
        motorSema.release();
        pulses = encoder.getPulses();
        encoder.reset();
        FILM_LENGTH = FILM_LENGTH - 5;
        cummulative_pulses = cummulative_pulses + pulses;
        update_film_value = true;
        update_sensor_data = true;
        flush_end.attach(&end_flush, 0.1);//timeout to update data
    }
 
    
}


void setup_flush_button(void) 
{
    Flush_button.mode(PullUp);
    Flush_button.attach_asserted(&start_flush);
    Flush_button.attach_deasserted(&end_flush);
    Flush_button.setSamplesTillAssert(10); // debounces 10 sample by
    Flush_button.setAssertValue(0);        // state of the pin
    Flush_button.setSampleFrequency();     // Defaults to 20ms.
    Flush_button.setSamplesTillHeld( 100 );
}


void SystemStates_thread(void const *name) 
{
  while (true) 
  {
 
    switch (sysState_struct.sysMode) 
    {
    case S_RUN:
      sysState_struct.sysMode = S_RUN;
    break;

    default:
      printf("Error: Undefined Mode\n");
      break;
    }
  }
}


void motorDrive_thread(void const *name) 
{
    while (true) 
    {
        motorSema.acquire();
        switch(gMotorAction)
        {
            case MA_Forward:
                motor.forward();
                VoltageThread.start(getVoltage);
            break;
            case MA_Backward:
                motor.backward();
            break;
            case MA_Brake:
                motor.brake();
                VoltageThread.terminate();
            break;
            case MA_Stop:
                motor.stop();
            break;
        }
    }
}



