#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ssd1306.h"
#include "standard_font.h"
SSD1306 oled;

int32_t plot[128],plot1[128],plot2[128],plot3[128],plot4[128],plot5[128],pmax,pmin;
uint32_t scale;
uint8_t pidx=0, didx=128;
extern uint8_t  power;
extern uint8_t  ina226;
extern uint64_t lsumBusMillVoltsOrig;
extern uint64_t lsumBusMillVolts;
extern int32_t  lmaxBusMillVolts;
extern int32_t  lminBusMillVolts;
extern int64_t  lsumBusMicroAmps;
extern int32_t  lmaxBusMicroAmps;
extern int32_t  lminBusMicroAmps;
extern int64_t  lsumBusMicroAmpsOrig;
extern int64_t  ltotalBusMicroAmps;
extern uint32_t lreadings;
extern int32_t  lnow;
extern osThreadId osUpdateScreenThreadId;
extern int16_t  zero;
extern int64_t  totalBusMicroAmps;
extern uint8_t  minRange;
extern bool serialEnable;
extern uint16_t ranges[4];
extern uint16_t voltageK;
extern uint16_t refreshT;
extern uint8_t  overload;

#define SERIALDEBUG
#ifdef SERIALDEBUG
#include <stdarg.h>
extern UART_HandleTypeDef huart1;
char debugstring[128];
int debugprintf (const char * format, ...) {
    va_list argptr;
    va_start (argptr, format);
    vsnprintf(debugstring,128,format,argptr);
    va_end(argptr);
    HAL_UART_Transmit(&huart1,(uint8_t *)debugstring,strlen(debugstring),5000);
    return 0;
}
#define _DEBUG(format, ...) debugprintf(format, __VA_ARGS__)
#else
#define _DEBUG(format, ...)
#endif


enum button{NOKEY,KEY1,KEY2,KEY3};
uint16_t buttonTime;
uint8_t buttonCode;
int8_t calibrationStep=0;
uint8_t gmode=0;

char sbuf[32];
void printFloat(float v, int8_t maxDigits, bool enlargeForMinus, const char* suffix) {
    float ov = v; v=abs(v);
    itoa((int)v,sbuf,10);
    int16_t i = maxDigits-strlen(sbuf);
    if (ov<0) { oled.putc('-'); i-=enlargeForMinus?0:1; }
    oled.puts(sbuf);
    if (i>0) {
        i = i==1?10:(i==2?100:1000);
        oled.putc('.');
        itoa(abs((int)(i*v))%i+i,sbuf,10); 
        oled.puts(sbuf+1);    
    }           
    oled.puts(suffix); 
}

extern "C" uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

extern "C" void updateScreenX(void const *arg);
void updateScreenX(void const *arg) {
  osUpdateScreenThreadId = osThreadGetId();

  oled.initialise();
  oled.clear();
  oled.set_contrast(255);  
  oled.set_font(bold_font, 8);  

  float v;

  for(;;)
  {
    osSignalWait(0x1,1000);    
    // code here normall executes every ~100ms
    uint8_t lbuttonCode=NOKEY;
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET)
      lbuttonCode = KEY1;
    else if (HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==GPIO_PIN_RESET)
      lbuttonCode = KEY2;
    else if (HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==GPIO_PIN_RESET)
      lbuttonCode = KEY3; 
    
    if (lbuttonCode==NOKEY && buttonCode!=NOKEY) {
      if (buttonCode==KEY1 && buttonTime>500) {
        // long KEY3 - reset stats
        totalBusMicroAmps = 0;
        lnow=0;
        didx=127;
        for (int i=0; i<128; i++) {
          plot[i]=0;
          plot1[i]=0;
          plot2[i]=0;
          plot3[i]=0;
          plot4[i]=0;
          plot5[i]=0;
        }
      } else if (buttonCode==KEY1) {
        // short KEY3 - toggle voltage o current grapth
          gmode = (gmode+1)%2;
      }

      if (buttonCode==KEY2 && buttonTime>500) {
        // long KEY2, toogle sending data
        serialEnable = !serialEnable;
      }

      if (buttonCode==KEY3) {
        // short KEY, toogle power
        if (power==1) 
          power=0;
        else
          calibrationStep=9;
        overload = 0;
      }
    }

    if (lbuttonCode != buttonCode) {
      buttonTime=0;
      buttonCode = lbuttonCode;
    }
    else 
      buttonTime+=100;

    // calibration
    if (calibrationStep>0) {
      switch (calibrationStep) {
        case 9:      // set 3 rd range
          minRange = 0;
          voltageK = 17000;    
          ina226 = 1;       
          HAL_GPIO_WritePin(ONEKLOAD_GPIO_Port, ONEKLOAD_Pin, GPIO_PIN_SET); 
          zero = lsumBusMicroAmpsOrig / lreadings;
          _DEBUG("Zero I %d\n",zero);     
          break;
        case 7:
          minRange = 3;
          ranges[3] = 10700;          
          // calibration current in ua on 1K is voltage in mv
          voltageK = (lsumBusMillVoltsOrig*voltageK)/lsumBusMillVolts;
          _DEBUG("INA226 %u mV, STM32 %u mV, INA226 %u uA\n",(uint)lsumBusMillVoltsOrig/lreadings,(uint)lsumBusMillVolts/lreadings,(uint)lsumBusMicroAmps/lreadings);
          _DEBUG("Voltage K %u\n",voltageK);
          break;
        case 5:
          minRange = 2;
          ranges[2] = 1222;
          ranges[3] = (lsumBusMillVoltsOrig*ranges[3])/lsumBusMicroAmpsOrig;
          _DEBUG("INA226 %u mV, STM32 %u mV, INA226 %u uA\n",(uint)lsumBusMillVoltsOrig/lreadings,(uint)lsumBusMillVolts/lreadings,(uint)lsumBusMicroAmps/lreadings);
          _DEBUG("Range 3 K %u\n",ranges[3]);   
          break;
        case 3:
          minRange = 0;
          power=1;
          ina226=0;
          HAL_GPIO_WritePin(ONEKLOAD_GPIO_Port, ONEKLOAD_Pin, GPIO_PIN_RESET);      
          ranges[2] = (lsumBusMillVoltsOrig*ranges[2])/lsumBusMicroAmps;
          _DEBUG("INA226 %u mV, STM32 %u mV, INA226 %u uA\n",(uint)lsumBusMillVoltsOrig/lreadings,(uint)lsumBusMillVolts/lreadings,(uint)lsumBusMicroAmps/lreadings);
          _DEBUG("Range 2 K %u\n",ranges[2]);             
          break;
        case 1:
          totalBusMicroAmps = 0;
          lnow=0;
          didx=127;
          for (int i=0; i<128; i++) {
            plot[i]=0;
            plot1[i]=0;
            plot2[i]=0;
            plot3[i]=0;
            plot4[i]=0;
            plot5[i]=0;
          }        
          calibrationStep = 0;
          break;
      }
      calibrationStep--;
    }

    // voltage
    oled.clear();
    v = (float)(lsumBusMillVolts/lreadings)/1000;
    oled.setCursor(0,0); 
    printFloat(v,3,false,"v");

    if (serialEnable) {
      oled.setCursor(6,0); 
      oled.putc(pidx%2?'^':' ');
    }

    // time

    itoa(lnow/3600000,sbuf,10);
    oled.setCursor(10-strlen(sbuf),0); 
    oled.puts(sbuf);
    oled.putc(':');  
    itoa(100+(lnow/60000)%60,sbuf,10);
    oled.puts(sbuf+1);
    oled.putc(':');  
    itoa(100+(lnow/1000)%60,sbuf,10);
    oled.puts(sbuf+1);              

    // graph
    plot[pidx]=lmaxBusMicroAmps;
    plot1[pidx]=lminBusMicroAmps; 
    plot2[pidx]=lsumBusMicroAmps/lreadings;
    plot3[pidx]=lmaxBusMillVolts;
    plot4[pidx]=lminBusMillVolts;
    plot5[pidx]=lsumBusMillVolts/lreadings;

    int32_t *lpmin, *lpmax, *lpavg;
    if (gmode==0) {
      lpmin = plot1;
      lpmax = plot;
      lpavg = plot2;
    } else {
      lpmin = plot4;
      lpmax = plot3;
      lpavg = plot5;
    }

    pmin=lpmin[pidx],pmax=lpmax[pidx];

    uint8_t idx_start = didx>0?((pidx-(127-didx))%128):(pidx+1)%128;
    for (uint8_t i=1; i<128-didx; i++) {
        uint8_t idx=(i+idx_start)%128;     
        if (lpmin[idx]<pmin)
            pmin = lpmin[idx];
        if (lpmax[idx]>pmax)
            pmax = lpmax[idx];
    }
    if (pmax==pmin) pmax+=1; // protect from device by zero
    if (power==1) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);    
      scale = ((1<<25)*36)/(pmax-pmin);   
      uint8_t idx_start = didx>0?((pidx-(127-didx))%128):(pidx+1)%128;
      for (uint8_t i=0,oidx=didx; i<128-didx; i++,oidx++) {
          uint8_t idx=(i+idx_start)%128;     
          uint8_t min = ((scale*(lpmin[idx]-pmin))>>25);
          uint8_t max = ((scale*(lpmax[idx]-pmin))>>25);        
          if (lpmax[idx]-lpmin[idx]<=(gmode==0?8:60)) {
            oled.set_pixel(oidx,52-((scale*(lpavg[idx]-pmin))>>25));
          } else {
            oled.line(oidx,53-min,oidx,52-max);
            if (max-min>4) {
              oled.clear_pixel(oidx,52-((scale*(lpavg[idx]-pmin))>>25));
            }
          }
      }  
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    

      pidx=pidx<127?pidx+1:0;
      if (didx>0) didx--;      

      // current mA
      oled.setCursor(0,1);
      printFloat((float)(lsumBusMicroAmps/lreadings)/1000,4,false,"mA");

      // max value on grath
      oled.setCursor(0,7);
      printFloat((float)pmin/1000,gmode==0?4:3,true,gmode==0?"mA":"v");

      oled.setCursor(9,7);
      printFloat((float)pmax/1000,gmode==0?4:3,false,gmode==0?"mA":"v");

      // mAh
      oled.setCursor(8,1);
      printFloat((float)(ltotalBusMicroAmps/lnow)/1000,4,false,"mAh");
    }

    if (overload) {
        oled.setCursor(3,3); 
        oled.puts("overload !!");
    }
    oled.update();
  } 
}