#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ssd1306.h"
#include "standard_font.h"
SSD1306 oled;

float plot[128],plot1[128],plot2[128], pmax,pmin,scale;
uint8_t pidx=0;
extern uint64_t lsumBusMillVolts;
extern int64_t  lsumBusMicroAmps;
extern int32_t  lmaxBusMicroAmps;
extern int32_t  lminBusMicroAmps;
extern int64_t  lsumBusMicroAmpsOrig;
extern int64_t  ltotalBusMicroAmps;
extern uint32_t lreadings;
extern int32_t  lnow;
extern osThreadId osUpdateScreenThreadId;
extern bool serialEnable;

char sbuf[32];
void printFloat(float v, int8_t maxDigits, bool enlargeForMinus, const char* suffix) {
    itoa((int)v,sbuf,10);
    int16_t i = maxDigits-strlen(sbuf);
    if (v<0) { oled.putc('-'); i-=enlargeForMinus?0:1; }
    oled.puts(sbuf);
    if (i>0) {
        i = i==1?10:(i==2?100:1000);
        oled.putc('.');
        itoa(abs((int)(i*v))%i+i,sbuf,10); 
        oled.puts(sbuf+1);    
    }           
    oled.puts(suffix); 
}

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
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);    

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
    oled.setCursor(8,0); 
    itoa(100+lnow/3600000,sbuf,10);
    oled.puts(sbuf+1);
    oled.putc(':');  
    itoa(100+lnow/60000,sbuf,10);
    oled.puts(sbuf+1);
    oled.putc(':');  
    itoa(100+(lnow/1000)%60,sbuf,10);
    oled.puts(sbuf+1);              

    // graph
    v = (float)(lsumBusMicroAmps/lreadings)/1000;
    plot[pidx]=(float)lmaxBusMicroAmps/1000;
    plot1[pidx]=(float)lminBusMicroAmps/1000;
    plot2[pidx]=v;
    pidx=pidx<127?pidx+1:0;
    pmin=plot1[0],pmax=plot[0];

    for (uint8_t i=0; i<128; i++) {
        if (plot1[i]<pmin)
            pmin = plot1[i];
        if (plot[i]>pmax)
            pmax = plot[i];
    }
    if (pmax==pmin) pmax=pmin+0.001; // protect from device by zero
    scale = 36.0/(pmax-pmin);   
    for (uint8_t i=0; i<128; i++) {
        uint8_t idx = (i+pidx)%128;
        uint8_t min = scale*(plot1[idx]-pmin)-1;
        uint8_t max = scale*(plot[idx]-pmin)+1;
        oled.line(i,54-min,i,54-max);
        if (max-min>4) {
          oled.clear_pixel(i,53-scale*(plot2[idx]-pmin));
        }
    }   

    // current mA
    oled.setCursor(0,1);
    printFloat(v,4,false,"mA");

    // max value on grath
    v = pmin;
    oled.setCursor(0,7);
    printFloat(v,4,true,"mA");

    v = pmax;
    oled.setCursor(9,7);
    printFloat(v,4,false,"mA");

    // mAh
    oled.setCursor(8,1);
    v = (float)(ltotalBusMicroAmps/lnow)/1000;
    printFloat(v,4,false,"mAh");

    oled.update();
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    
  } 
}