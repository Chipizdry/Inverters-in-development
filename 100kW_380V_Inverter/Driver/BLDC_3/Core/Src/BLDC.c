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

/*
void Phaze_A_ON(uint16_t pwm){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); TIM1->CCR1 =pwm;HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);}
void Phaze_A_OFF(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);TIM1->CCR1 =TIM1->ARR; HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);}
void Phaze_A_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);}
void Phaze_A_PWM(uint16_t pwm){HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);TIM1->CCR1 =pwm; }
void Phaze_A_LOW(void){TIM1->CCR1 = TIM1->ARR; HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);}// 0% duty → всегда LOW-side
void Phaze_A_HIGH(void){TIM1->CCR1 = TIM1->ARR;} // 100% duty → всегда HIGH-side


void Phaze_B_ON(uint16_t pwm){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);TIM1->CCR2 =pwm;  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);}
void Phaze_B_OFF(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);TIM1->CCR2 =TIM1->ARR; HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); }
void Phaze_B_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);}
void Phaze_B_PWM(uint16_t pwm){HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);TIM1->CCR2 =pwm;}
void Phaze_B_LOW(void){TIM1->CCR2 = TIM1->ARR; HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);}// 0% duty → всегда LOW-side
void Phaze_B_HIGH(void){TIM1->CCR2 = TIM1->ARR;} // 100% duty → всегда HIGH-side


void Phaze_C_ON(uint16_t pwm){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);TIM1->CCR3 =pwm;HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  }
void Phaze_C_OFF(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);TIM1->CCR3 =TIM1->ARR;HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);}
void Phaze_C_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);}
void Phaze_C_PWM(uint16_t pwm){HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);TIM1->CCR3 =pwm;}
void Phaze_C_LOW(void){TIM1->CCR3 = TIM1->ARR; HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);}// 0% duty → всегда LOW-side
void Phaze_C_HIGH(void){TIM1->CCR3 = TIM1->ARR;} // 100% duty → всегда HIGH-side
*/

/**
  * @brief  Коммутация фаз BLDC-двигателя (режим мотора)
  * @param  halls: комбинация сигналов датчиков Холла (5,1,3,2,6,4)
  * @param  pulse: значение ШИМ для верхнего ключа (0...TIM1->ARR)
  * @retval None
  */
void BLDC_MotorCommutation(uint8_t halls, uint16_t pulse) {
    // 1. Безопасное состояние – отключаем все выходы таймера
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);


    // 2. Логика коммутации (PWM mode 1: CCR=pulse → верхний ШИМ; CCR=0 → нижний постоянно включён)
    switch (halls) {


        case 1: // Шаг 2: A+ (PWM), C- (ON), B (ZZ)
            TIM1->CCR1 = pulse;
            TIM1->CCR3 = TIM1->ARR; ;
            TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC3NE);
            break;

        case 2: // Шаг 4: B+ (PWM), A- (ON), C (ZZ)
		   TIM1->CCR2 = pulse;
		   TIM1->CCR1 = TIM1->ARR; ;
		   TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC1NE);
		   break;

        case 3: // Шаг 3: B+ (PWM), C- (ON), A (ZZ)
            TIM1->CCR2 = pulse;
            TIM1->CCR3 = TIM1->ARR; ;
            TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3NE);
            break;
        case 4: // Шаг 6: C+ (PWM), B- (ON), A (ZZ)
		  TIM1->CCR3 = pulse;
		  TIM1->CCR2 = TIM1->ARR; ;
		  TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC2NE);
		  break;

        case 5: // Шаг 1: A+ (PWM), B- (ON), C (ZZ)
		  TIM1->CCR1 = pulse;
		  TIM1->CCR2 = TIM1->ARR; ;
		  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2NE);
		  break;

        case 6: // Шаг 5: C+ (PWM), A- (ON), B (ZZ)
            TIM1->CCR3 = pulse;
            TIM1->CCR1 = TIM1->ARR; ;
            TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC1NE);
            break;


        default: // Некорректная комбинация – всё отключаем
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
            break;
    }
}

/**
  * @brief  Коммутация фаз BLDC-генератора (режим рекуперации)
  * @param  halls: комбинация датчиков (те же значения)
  * @param  pulse: значение ШИМ для верхнего ключа
  * @retval None
  */
void BLDC_GeneratorCommutation(uint8_t halls, uint16_t pulse) {
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);


    switch (halls) {
        case 1: // A+ PWM, B ZZ, C LOW
            TIM1->CCR1 = pulse;
            TIM1->CCR3 = 0;
            TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC3NE);
            break;
        case 2: // A LOW, B+ PWM, C ZZ
            TIM1->CCR2 = pulse;
            TIM1->CCR1 = 0;
            TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC1NE);
            break;
        case 3: // A ZZ, B+ PWM, C LOW
            TIM1->CCR2 = pulse;
            TIM1->CCR3 = 0;
            TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3NE);
            break;
        case 4: // A ZZ, B LOW, C+ PWM
            TIM1->CCR3 = pulse;
            TIM1->CCR2 = 0;
            TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC2NE);
            break;
        case 5: // A+ PWM, B LOW, C ZZ
            TIM1->CCR1 = pulse;
            TIM1->CCR2 = 0;
            TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2NE);
            break;
        case 6: // A LOW, B ZZ, C+ PWM
            TIM1->CCR3 = pulse;
            TIM1->CCR1 = 0;
            TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC1NE);
            break;
        default: // 0,7 – всё отключаем
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
            break;
    }
}



/*
void BLDC_MotorCommutation(uint8_t halls  ){

	 switch (halls) {

	case 1:


	Phaze_B_ZZ();
	Phaze_C_OFF();
	Phaze_A_ON();

	break;
	case 2:

    Phaze_A_OFF();
	Phaze_B_ON();
    Phaze_C_ZZ();

	break;
	case 3:

	Phaze_A_ZZ();
	Phaze_B_ON();
	Phaze_C_OFF();

	break;
	case 4:

	Phaze_A_ZZ();
	Phaze_B_OFF();
	Phaze_C_ON();


	break;
	case 5:

	Phaze_A_ON();
	Phaze_B_OFF();
	Phaze_C_ZZ();

	break;
	case 6:

	Phaze_A_OFF();
	Phaze_B_ZZ();
	Phaze_C_ON();

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
*/

/*
void BLDC_MotorCommutation(uint8_t halls, uint16_t pwm)
{
    // Ограничиваем pwm значением периода (ARR)
  //  if (pwm > TIM1->ARR) pwm = TIM1->ARR;


    switch (halls)
    {
        case 1:  // Состояние Холлов: A=0, B=0, C=1 (пример)
        	TIM1->CCR1 =0;
        	TIM1->CCR2 =0;
        	TIM1->CCR3 =0;
            Phaze_B_ZZ();               // Фаза B – отключена
            Phaze_C_LOW();              // Фаза C – нижний ключ постоянно включён
           // Phaze_A_PWM(pwm);           // Фаза A – верхний ключ с ШИМ
            Phaze_A_ON(pwm);
            break;

        case 2:  // A=0, B=1, C=0
        	TIM1->CCR1 =0;
        	        	TIM1->CCR2 =0;
        	        	TIM1->CCR3 =0;
            Phaze_C_ZZ();
            Phaze_A_LOW();
          //  Phaze_B_PWM(pwm);
            Phaze_B_ON(pwm);
            break;

        case 3:  // A=0, B=1, C=1
        	TIM1->CCR1 =0;
        	TIM1->CCR2 =0;
        	TIM1->CCR3 =0;
            Phaze_A_ZZ();
            Phaze_C_LOW();
            //Phaze_B_PWM(pwm);
            Phaze_B_ON(pwm);
            break;

        case 4:  // A=1, B=0, C=0
            Phaze_B_ZZ();
            Phaze_A_LOW();
          //  Phaze_C_PWM(pwm);
            Phaze_C_ON(pwm);
            break;

        case 5:  // A=1, B=0, C=1
        	TIM1->CCR1 =0;
			TIM1->CCR2 =0;
			TIM1->CCR3 =0;
            Phaze_C_ZZ();
            Phaze_B_LOW();
           // Phaze_A_PWM(pwm);
            Phaze_A_ON(pwm);
            break;

        case 6:  // A=1, B=1, C=0
        	TIM1->CCR1 =0;
        	TIM1->CCR2 =0;
        	TIM1->CCR3 =0;
            Phaze_A_ZZ();
            Phaze_C_LOW();
          //  Phaze_B_PWM(pwm);
            Phaze_B_ON(pwm);
            break;

        default: // 0 или 7 – недопустимые комбинации (все датчики в 0 или 1)
            Phaze_A_ZZ();
            Phaze_B_ZZ();
            Phaze_C_ZZ();
            break;
    }
}


void BLDC_GeneratorCommutation(uint8_t halls ,uint16_t pwm ){

	 switch (halls) {

	case 1:

    Phaze_A_PWM(pwm);
	Phaze_B_ZZ();
	Phaze_C_LOW();

	break;
	case 2:

    Phaze_A_LOW();
	Phaze_B_PWM(pwm);
    Phaze_C_ZZ();

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

	Phaze_A_PWM(pwm);
	Phaze_B_LOW();
	Phaze_C_ZZ();

	break;
	case 6:

	Phaze_A_LOW();
	Phaze_B_ZZ();
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

*/
