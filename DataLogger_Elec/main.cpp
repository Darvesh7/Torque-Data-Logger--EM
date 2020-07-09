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
#include <string>
using std::string;


#include "platform/mbed_retarget.h"

//////////////////////////
/////// Buttons ////////// 
/////////////////////////

Ds3231 rtc(PB_9, PB_8);
SDBlockDevice sd(SPI_MOSI, SPI_MISO, SPI_SCK, PA_9);
FATFileSystem fs("sd", &sd);

time_t epoch_time;
Timer t;

PinDetect Flush_button (PC_13);
Motor motor(PA_10,PB_3,PB_5,PB_4,PA_4);
QEI encoder(PC_1,PC_0,NC,2338);
AnalogIn voltagePin(PA_0);
ACS712 CurrentSensor(PA_1,1,30);

//////////////////////////
//////// Variables ///////
/////////////////////////


bool update_film_value = false;
bool update_rpm_value = false;

int flush_count = 0;
volatile float cummulative_rpm = 0.0;

//////////////////////////
////////////RTOS ///////// 
/////////////////////////

Semaphore motorSema(1);
Semaphore loggerSema(1);

void SystemStates_thread(void const *name);
void motorDrive_thread(void const *name); 
void logger(void const *name);

Thread t1;
Thread t2;
Thread loggerThread;

EventFlags stateChanged;
EventQueue queue;
Timeout flush_end;

FILE * fd;


////////////////////////
/////RPM Variables////// 
///////////////////////
unsigned long start = 0;
double ppr = 16; 
uint8_t upDatesPerSec = 2;
const int fin = 1000 / upDatesPerSec;
const float constant = 60.0 * upDatesPerSec / (ppr*2);


void start_flush(void);
void end_flush(void);
void setup_flush_button(void);
void getTorque(void);
void getRpm(void);
void getVoltage(void);
void getCurrent(void);
void getPower(void);

void savetoSD(void);

Serial pc(USBTX, USBRX);


void setup()
{
   
    pc.baud(115200);
    
    sd.init();
    fs.mount(&sd);


    fd = fopen("/sd/testdata.txt", "r+");
    if (fd != nullptr)
    {
        pc.printf("SD Mounted - Ready");

    }
    else
    {
        fd = fopen("/sd/testdata.txt", "w+");
        fclose(fd);
        pc.printf("File Closed\r\n");
        sd.deinit();
        fs.unmount();
        pc.printf("Creating new file - No existing file found\r\n");
    }

    setup_flush_button();

    t1.start(callback(motorDrive_thread, (void *)"MotorDriveThread"));
    t2.start(callback(SystemStates_thread, (void *)"SystemStateThread"));
    loggerThread.start(callback(logger, (void *)"DataLoggerThread"));

    sysState_struct.sysMode = S_RUN;
  
}




int main()

{

  
    setup(); 

    while (true) 
    {

    }

}

bool sdopened = false;

void logger(void const *name)
{
    volatile int currenttime = 0;
    volatile float voltage = 0.0;
    volatile float current = 0.0;
    volatile int pulses = 0;
    
    while(true)
    {
        if(loggerSema.try_acquire_until(20000))
        {
            if (!sdopened)
            {
                sd.init();
                fs.mount(&sd);
                fd = fopen("/sd/testdata.txt", "a");
                sdopened = (fd != nullptr);
            }

            voltage = voltagePin.read_u16() * (16.70 / 65535.00);
            current = CurrentSensor;
            pulses = encoder.getPulses();
            currenttime = t.read_ms();

            string logline;

            logline.append(to_string(voltage) + ",");
            logline.append(to_string(current) + ",");
            logline.append(to_string(pulses) + ",");
            logline.append(to_string(currenttime) + "\n");

            pc.printf("%s", logline.c_str());

            if (fd != nullptr)
            {
                fprintf(fd, "%s", logline.c_str());
            }

            encoder.reset();
            t.reset();
            ThisThread::sleep_for(100);
        
            if(motor._MState == MFORWARD)
            {
                loggerSema.release();
            }
            else
            {
                if (fd != nullptr)
                {
                    fflush(stdout);
                    fclose(fd);
                    sd.deinit();
                    fs.unmount();
                    sdopened = false;
                }
            }
        }
    }
}

// void getPower()
// {
//     while(true)
//     {
//     Power = (Voltage * Current);
//     //Power = voltagePin.read_u16()*(16.70/65535.00) * float(CurrentSensor);
//     }
//     ThisThread::sleep_for(500);

// }
       
// void getRpm(void)
// {
    
//     while(true)
//     {

//             //rpmTimer.start();
//             //if (rpmTimer.read_ms() - start > fin) 
//             //{
//             //start = rpmTimer.read_ms();
        
 //            rpm = abs((encoder.getPulses() * (constant)/149.25));
//             //printf("RPM = %2.2f\n", (rpm));
    
//             encoder.reset();
//             //cummulative_rpm = cummulative_rpm + rpm;
//             //rpmTimer.reset();     
//             //} 

//         ThisThread::sleep_for(500);          
//     }
    
   
// }

// void getTorque(void)
// {   
//     while(true)
//     {

//         Torque = Power/(2*(2*3.14)*rpm);   
//         ThisThread::sleep_for(500);

//     }

// } 


void start_flush(void)
{
    //VoltageThread.start(getVoltage);
    if(sysState_struct.sysMode == S_RUN)
    {
        if(motor._MState == MSTOP)
        {
            gMotorAction = MA_Forward;
            t.start();
            motorSema.release();
            loggerSema.release();
            flush_end.attach(&end_flush, 4.0); //timeout - duration of flush
       
        }
    }
}

void end_flush(void)
{
    if(motor._MState == MFORWARD)
    {
        gMotorAction = MA_Brake;
        motorSema.release();
        flush_count = flush_count + 1;
        update_film_value = true;  
        t.stop(); 
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

