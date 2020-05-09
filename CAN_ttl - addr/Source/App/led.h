 
#include "stm32f10x.h"
#include "ALL_Includes.h"
#include "BitBand.h"
extern unsigned char testxx;

  
#define led1      PBOUT(8)
#define led2      PBOUT(6)
#define led3      PBOUT(7)
 
#define ctrl_485  PCOUT(13)
#define en_485_send  ctrl_485=1;
#define en_485_rec  ctrl_485=0;


 






 
