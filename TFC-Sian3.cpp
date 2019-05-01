#include "mbed.h"
#include "TFC.h"


//Serial pc(USBTX, USBRX);

float servoTrim();
float motorTrim();
float avgPos();
int readCamCenter();

int cam[122],
    peaks[4];

int filter[]={-1,-1,-1,6,-1,-1,-1}; //size must not change

float servBuf[100] = {0},
      weight[100];

int main(){
    float pos = 0,
          speed = 0;
          
    int prevState1 = 0,
        prevState2 = 0;
        
        
    weight[0] = 1;
    for(int i = 1; i < 100; i++){
        weight[i] = weight[i-1] * 0.95;
    }

    //pc.baud(115200);
    TFC_Init();
    
    while(TFC_ReadPushButton(0) == 0){
    }
    
    wait(3);
    
    TFC_HBRIDGE_ENABLE;
    
    while(1)
    {        
        if(TFC_ReadPushButton(0) != 0 && prevState1 != TFC_ReadPushButton(0))
        {
            speed -= 0.20;
        }
        else if(TFC_ReadPushButton(1) != 0 && prevState2 != TFC_ReadPushButton(1))
        {
            speed += 0.20;
        }
        
        prevState1 = TFC_ReadPushButton(0);
        prevState2 = TFC_ReadPushButton(1);
        
        TFC_SetMotorPWM(speed, speed);
           
        if(((peaks[0]>40 && peaks[0]<61) /*&& (pos<0 || pos>-1)*/) || ((peaks[1]>40 && peaks[1]<61)  /*&& (pos<0 || pos>-1)*/)) //if 1st and 2nd peak move towards center, steer right. 
        {
            
            TFC_SetServo(0, 0.5); // TFC_SetServo(servo, pos) while  pos=50.5/122 or 37.475degree
            
            
        }
        else if(((peaks[2]<82 && peaks[2]>61) && (pos>0 || pos<1)) || ((peaks[3]<82 && peaks[3]>61) /*&& (pos>0 || pos<1)*/)) //if 3rd and 4th peak move towards center, steer left.
        {   
        
            TFC_SetServo(0, -0.5);
            
        
        }
        else //Other than all these case, just go straight.
        {
                       
            
            //pos = 0;
            TFC_SetServo(0, 0);
        }
            
        pos = float(readCamCenter() - 30.5) / 30.5;
    
        pos *= (TFC_ReadPot(1) + 1) / 2;
        
        pos += TFC_ReadPot(0);
        
        if((TFC_ServoTicker % 5) == 0)
        {
            for(int i = 99; i > 0; i--)
            {
                servBuf[i] = servBuf[i-1];
            }
            servBuf[0] = pos;
        }
    
        TFC_SetServo(0, 0);
    
        
    
    }
}

float servoTrim(){
    return TFC_ReadPot(0);
}

float motorTrim(){
    return TFC_ReadPot(1);
}

int readCamCenter(){
    int temp;
    int peaksval[] ={0, 0, 0, 0};
    
    if(TFC_LineScanImageReady != 0){
        
        TFC_LineScanImageReady = 0;
        for(int i = 0; i < 122; i++){
            temp = 0;
            for(int h = 0; h < 7; h++){
                temp = temp + (TFC_LineScanImage0[i+h] * filter[h]);
            }
            cam[i] = temp;
        }
    }

    temp = 0;
    
    for(int i = 0; i < 4; i++){
        peaksval[i] = 0;
    }
    
    for(int i = 0; i < 122; i++) //find 1st peak or peak[0] position
    {
        if(cam[i] > peaksval[0])
        {
            peaksval[0] = cam[i];
            peaks[0] = i;
        }
    }
    
    for(int i = 0; i < 122; i++) //find 2nd peak or peak[1] position
    { 
        if(cam[i] > peaksval[1] && peaks[0] != i)
        {
            peaksval[1] = cam[i];
            peaks[1] = i;
        }
    }
    
    for(int i = 0; i < 122; i++) //find 3rd peak or peak[2] position
    { 
        if(cam[i] > peaksval[1] && peaks[0] != i && peaks[1] != i)
        {
            peaksval[2] = cam[i];
            peaks[2] = i;
        }
    }
    
    for(int i = 0; i < 122; i++) //find 4th peak or peak[3] position
    { 
        if(cam[i] > peaksval[1] && peaks[0] != i && peaks[1] != i && peaks[2] != i)
        {
            peaksval[3] = cam[i];
            peaks[3] = i;
        }
    }

    temp = ((peaks[0]+1)+(peaks[1]+1)+(peaks[2]+1)+(peaks[3]+1))/4; //find the center of the track
    
    return  temp;
}

float avgPos(){
    float temp = 0;
    
    for(int i = 0; i < 100; i++){
        temp += servBuf[i] * weight[i];
    }
    
    return (temp / 19.8816);
}
