#include "mbed.h"
#include "motor.h"
#include "QEI.h"
#include "PinDetect.h"
#include "ACS712.h"
#include "states.h"
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include "ds3231.h"
#include <stdio.h>
#include <errno.h>


#include "platform/mbed_retarget.h"

//////////////////////////
/////// Buttons ////////// 
/////////////////////////

Ds3231 rtc(PB_9, PB_8);
SDBlockDevice sd(SPI_MOSI, SPI_MISO, SPI_SCK, PA_9);
FATFileSystem fs("sd", &sd);


FILE *fd;
time_t epoch_time;

PinDetect Flush_button (PC_13);
Motor motor(PA_10,PB_3,PB_5,PB_4,PA_4);
QEI encoder(PC_1,PC_0,NC,2338);
AnalogIn voltagePin(PA_0);
ACS712 CurrentSensor(PA_1,1,30);


//////////////////////////
//////// Variables ///////
/////////////////////////


bool update_film_value = false;
bool update_sensor_data = false;

int pulses = 0;
int flush_count = 0;

volatile float Voltage = 0.0;
volatile float Current = 0.0;
volatile float Power = 0.0;
volatile float Torque = 0.0;

//////////////////////////
////////////RTOS ///////// 
/////////////////////////

Semaphore motorSema(1);

void SystemStates_thread(void const *name);
void motorDrive_thread(void const *name); 

Thread t1;
Thread t2;
Thread VoltageThread, CurrentThread, PowerThread, RpmThread, TorqueThread;

EventFlags stateChanged;
EventQueue queue;
Timeout flush_end;
Timer rpmTimer;


////////////////////////
/////RPM Variables////// 
///////////////////////
unsigned long start = 0;
double ppr = 16; 
uint8_t upDatesPerSec = 2;
const int fin = 1000 / upDatesPerSec;
const float constant = 60.0 * upDatesPerSec / (ppr*2);
float rpm;

void start_flush(void);
void end_flush(void);
void setup_flush_button(void);
void getTorque(void);
void getRpm(void);



void setup()
{
    epoch_time = rtc.get_epoch();

    fs.mount(&sd);

    printf("Opening a new file, numbers.txt.");
    fd = fopen("/sd/testdata.txt", "w+");
    fprintf(fd, "%s\n",  ctime(&epoch_time));

    fclose(fd);

    setup_flush_button();

    t1.start(callback(motorDrive_thread, (void *)"MotorDriveThread"));
    t2.start(callback(SystemStates_thread, (void *)"SystemStateThread"));

    sysState_struct.sysMode = S_RUN;
  
}


int main()

{
  
    setup();

    while (true) 
    {
    sleep();
    
        if (update_film_value) 
        {

        printf("Flush Count %d\n", flush_count);
        pulses = 0;
        encoder.reset();
        
        update_film_value = false;
        }

    }
}



void getVoltage()
{
    Voltage = voltagePin.read_u16()*(16.70/65535.00); 

}

void getCurrent()
{
    Current = float(CurrentSensor);

}

void getPower()
{
    Power = voltagePin.read_u16()*(16.70/65535.00) * float(CurrentSensor);

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
   
}

void getTorque(void)
{   
    Torque = Power/(2*(2*3.14)*rpm);   
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
        gMotorAction = MA_Stop;
        motorSema.release();
        flush_count = flush_count + 1;
        update_film_value = true;   
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
               
            break;
            case MA_Backward:
                motor.backward();
            break;
            case MA_Brake:
                motor.brake();
            break;
            case MA_Stop:
                motor.stop();
            break;
        }
    }
}



