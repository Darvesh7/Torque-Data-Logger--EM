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

InterruptIn Flush_button (PC_13);
Motor motor(PA_10,PB_3,PB_5,PB_4,PA_4);
QEI encoder(PC_1,PC_0,NC,2338);
AnalogIn voltagePin(PA_0);
ACS712 CurrentSensor(PA_1,1,30);




time_t epoch_time;
LowPowerTimer t;


//////////////////////////
//////// Variables ///////
/////////////////////////


bool update_film_value = false;
bool update_rpm_value = false;

int flush_count = 0;


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
LowPowerTimeout  flush_end;

FILE * fd;


void start_flush(void);
void end_flush(void);
void setup_flush_button(void);
void getTorque(void);
void getRpm(void);
void getVoltage(void);
void getCurrent(void);
void getPower(void);

void SetSysClock_PLL_HSE(void);

void savetoSD(void);

Serial pc(USBTX, USBRX);


void init_SD(void)
{
    pc.baud(115200);

    sd.init();
    fs.mount(&sd);

   
    fd = fopen("/sd/testdata.csv", "r+");
    if (fd != nullptr)
    {
        pc.printf("SD Mounted - Ready");

    }
    else
    {
        fd = fopen("/sd/testdata.csv", "w+");
        fclose(fd);
        pc.printf("File Closed\r\n");
        sd.deinit();
        fs.unmount();
        pc.printf("Creating new file - No existing file found\r\n");
    }
}

void setup()
{
    pc.printf("SystemCoreClock is %d Hz\r\n", SystemCoreClock);


    setup_flush_button();

    t1.start(callback(motorDrive_thread, (void *)"MotorDriveThread"));
    t2.start(callback(SystemStates_thread, (void *)"SystemStateThread"));
    loggerThread.start(callback(logger, (void *)"DataLoggerThread"));

    sysState_struct.sysMode = S_RUN;
  
}

void LowPowerConfiguration(void)
{
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN |
                    RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOEEN |RCC_AHBENR_GPIOHEN);
                    
    GPIO_InitTypeDef GPIO_InitStruct;
    // All other ports are analog input mode
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    //HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    
    RCC->AHBENR &= ~(RCC_AHBENR_GPIOBEN  |RCC_AHBENR_GPIODEN|
                    RCC_AHBENR_GPIODEN| RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOHEN);
}



int main()

{
    
    init_SD();
    setup(); 
    Flush_button.fall(&start_flush);
    Flush_button.rise(&end_flush);  

    while (true) 
    {
        if(update_film_value)
        {
            HAL_Delay(1000); //important to have Delay - > Give SD time to flush data.
            update_film_value = false;
            LowPowerConfiguration();
            /* Enter Stop Mode */
            HAL_SuspendTick();
            HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

            SetSysClock();

            HAL_ResumeTick();

            AnalogIn voltagePin(PA_0);
            ACS712 CurrentSensor(PA_1,1,30);
            setup();       
            pc.printf("exiting sleep mode");
        }          
   
    }

}








bool sdopened = false;

void logger(void const *name)
{
    volatile int currenttime = 0;
    volatile float voltage = 0.0;
    volatile float current = 0.0;
    volatile int pulses = 0;
    volatile float rpm = 0.0;
    float RpmRatioConversion = ((600/32)*(1/149.25));
    volatile float power = 0.0;
    volatile float torque = 0.0;


    
    while(true)
    {
        
        epoch_time = rtc.get_epoch();
        if(loggerSema.try_acquire_until(20000))
        {
            if (!sdopened)
            {
                sd.init();
                fs.mount(&sd);
                fd = fopen("/sd/testdata.csv", "a");
                fflush(stdout);
                fprintf(fd, "%s\n", ctime(&epoch_time));
                sdopened = (fd != nullptr);
            }

            voltage = voltagePin.read_u16() * (16.70 / 65535.00);
            current = abs(CurrentSensor);
            pulses = encoder.getPulses();
            currenttime = t.read_ms();
            rpm = (pulses*RpmRatioConversion);
            power = voltage*current;
            torque = ((power/(2*3.14*rpm)) * 60);


            string logline;

            logline.append(to_string(flush_count) + ",");
            logline.append(to_string(voltage) + ",");
            logline.append(to_string(current) + ",");
            logline.append(to_string(power)  + ",");
            logline.append(to_string(pulses) + ",");
            logline.append(to_string(currenttime) + ",");
            logline.append(to_string(rpm)  + ",");
            logline.append(to_string(torque) + "\n");

            pc.printf("%s", logline.c_str());

            if (fd != nullptr)
            {
                
                fprintf(fd, " %s%s",",",logline.c_str());
    
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
                    fprintf(fd, "\r\n");
                    fclose(fd);
                    sd.deinit();
                    fs.unmount();
                    sdopened = false;
                }
            }
        }
    }
}


void start_flush(void)
{
    //VoltageThread.start(getVoltage);
    if(sysState_struct.sysMode == S_RUN)
    {
        if(motor._MState == MSTOP)
        {
            
            flush_count = flush_count + 1;
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
        t.stop(); 
       
        update_film_value = true;  
        flush_end.attach(&end_flush, 0.1); //timeout - duration of flush
    }
   
}


void setup_flush_button(void) 
{
    //Flush_button.mode(PullUp);
    //Flush_button.attach_asserted(&start_flush);
    //Flush_button.attach_deasserted(&end_flush);
    //Flush_button.setSamplesTillAssert(10); // debounces 10 sample by
    //Flush_button.setAssertValue(0);        // state of the pin
    //Flush_button.setSampleFrequency();     // Defaults to 20ms.
    //Flush_button.setSamplesTillHeld( 100 );
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