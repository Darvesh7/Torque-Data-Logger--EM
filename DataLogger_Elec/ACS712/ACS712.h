#include <mbed.h>

class ACS712 { 
    
    public:
 
       
    ACS712(PinName _pin, float voltDivRatio = 1, short type = 5);
    
    float read();

    ACS712& operator= (const ACS712&);

    operator float() 
    {           
    return read();        
    }
            
    private:
        
    AnalogIn sensor;
    float translate(float);
    float ratio;
    short type;

};

ACS712::ACS712(PinName _pin, float voltDivRatio, short sensorType): 
sensor(_pin)
{
    ratio = voltDivRatio;
    type = sensorType;    
}

float ACS712::translate(float val)
{
    switch(type){
        case 5: 
            return (val*ratio - 2.44*ratio)/(.185*ratio);
        case 20:
            return (val*ratio - 2.44*ratio)/(.1*ratio);
        case 30:
            return (val*ratio - 2.38*ratio)/(.066*ratio);
        default:
            return 999;
    }
}


float ACS712::read()
{
    return ACS712::translate(sensor * 3.3);   
}

ACS712& ACS712::operator=(const ACS712& rhs)
{
    sensor = rhs.sensor;
    ratio = rhs.ratio;
    type = rhs.type;
    return *this;
}
