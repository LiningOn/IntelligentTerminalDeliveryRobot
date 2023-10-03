#ifndef __SPEED_ANALYSIS_H
#define __SPEED_ANALYSIS_Hvoid Speed_analysis(void)

void Speed_analysis(void);
void Encoder_Get_CNT(void);
void Set_PWM(void);
void Speed_programme(int time, float v_max, float acc_pro, float dec_pro, float *speed);

#endif
