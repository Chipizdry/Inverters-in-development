

/*
 * BLDC.c
 *
 *  Created on: Jun 20, 2024
 *      Author: chipi
 */
#include "BLDC.h"

uint8_t SWITCH( bool coils[]){

uint8_t connector=0;

connector|=coils[0]&1;
connector|=(coils[1]&1)<<1;
connector|=(coils[2]&1)<<2;
return connector;
}

void Phaze_A_ON(uint16_t pwm){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);TIM1->CCR1=pwm; HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);}
void Phaze_A_OFF(uint16_t pwm){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); TIM1->CCR1=pwm;HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);}
void Phaze_A_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);}
void Phaze_A_LOW(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);TIM1->CCR1=TIM1->ARR;HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);}
void Phaze_A_PWM(uint16_t pwm){TIM1->CCR1=TIM1->ARR-pwm;HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);}

void Phaze_B_ON(uint16_t pwm){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);TIM1->CCR2=pwm;  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);}
void Phaze_B_OFF(uint16_t pwm){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);TIM1->CCR2=pwm; HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); }
void Phaze_B_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);}
void Phaze_B_LOW(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);TIM1->CCR2=TIM1->ARR;HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);}
void Phaze_B_PWM(uint16_t pwm){TIM1->CCR2=TIM1->ARR-pwm;HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);}

void Phaze_C_ON(uint16_t pwm){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);TIM1->CCR3=pwm;HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  }
void Phaze_C_OFF(uint16_t pwm){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);TIM1->CCR3=pwm; HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);}
void Phaze_C_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);}
void Phaze_C_LOW(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);TIM1->CCR3=TIM1->ARR;HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);}
void Phaze_C_PWM(uint16_t pwm){TIM1->CCR3=TIM1->ARR-pwm;HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);}

void BLDC_MotorCommutation(uint8_t halls, uint16_t pwm){
	 TIM1->CCR1=pwm;
	 TIM1->CCR2=pwm;
	 TIM1->CCR3=pwm;

	 switch (halls) {


	case 1:
	Phaze_B_ZZ();
    Phaze_A_ON(pwm);
	Phaze_C_OFF(pwm);
	break;

	case 2:
    Phaze_C_ZZ();
    Phaze_A_OFF(pwm);
	Phaze_B_ON(pwm);
	break;

	case 3:
	Phaze_A_ZZ();
	Phaze_B_ON(pwm);
	Phaze_C_OFF(pwm);
	break;

	case 4:
	Phaze_A_ZZ();
	Phaze_B_OFF(pwm);
	Phaze_C_ON(pwm);
	break;

	case 5:
	Phaze_C_ZZ();
	Phaze_A_ON(pwm);
	Phaze_B_OFF(pwm);
	break;

	case 6:
	Phaze_B_ZZ();
	Phaze_A_OFF(pwm);
	Phaze_C_ON(pwm);
	break;

	case 7:
	Phaze_A_ZZ();
	Phaze_B_ZZ();
	Phaze_C_ZZ();
	break;

	case 0:
	Phaze_A_ZZ();
	Phaze_B_ZZ();
	Phaze_C_ZZ();
	break;

	default:
	break;}

}



void BLDC_GeneratorCommutation(uint8_t halls, uint16_t pwm){

	 TIM1->CCR1=pwm;
	 TIM1->CCR2=pwm;
	 TIM1->CCR3=pwm;

	 switch (halls) {

	case 1:
	Phaze_B_ZZ();
	Phaze_A_PWM(pwm);
	Phaze_C_LOW();
	break;

	case 2:
    Phaze_C_ZZ();
    Phaze_A_LOW();
    Phaze_B_PWM(pwm);
	break;

	case 3:
	Phaze_A_ZZ();
	Phaze_B_PWM(pwm);
	Phaze_C_LOW();
	break;

	case 4:
	Phaze_A_ZZ();
	Phaze_B_LOW();
	Phaze_C_PWM(pwm);
	break;

	case 5:
	Phaze_C_ZZ();
	Phaze_A_PWM(pwm);
	Phaze_B_LOW();
	break;

	case 6:
	Phaze_B_ZZ();
    Phaze_A_LOW();
	Phaze_C_PWM(pwm);
	break;

	case 7:
	Phaze_A_ZZ();
	Phaze_B_ZZ();
	Phaze_C_ZZ();
	break;

	case 0:
	Phaze_A_ZZ();
	Phaze_B_ZZ();
	Phaze_C_ZZ();
	break;

	default:
	break;}

}
