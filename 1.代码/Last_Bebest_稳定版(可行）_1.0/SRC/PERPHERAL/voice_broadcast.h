#ifndef __VOICE_BROADCAST_H
#define __VOICE_BROADCAST_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


extern u8 Next_song[4];
extern u8 Last_song[4];
extern u8 voice_1[6];
extern u8 voice_2[6];
extern u8 voice_3[6];
extern u8 voice_4[6];
extern u8 voice_5[6];
extern u8 voice_6[6];
extern u8 voice_7[6];
extern u8 voice_8[6];
extern u8 voice_9[6];
extern u8 voice_10[6];
extern u8 voice_11[6];

extern u8 DATA[5];

extern u8 open[4];
extern u8 stop[4];

void Send_data(int num,u8 *Data);

#endif



