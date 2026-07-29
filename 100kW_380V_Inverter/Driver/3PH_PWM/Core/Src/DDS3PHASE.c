



#include "DDS3PHASE.h"
#include <math.h>

DDS3PHASE_t DDS;

int16_t SinTable[DDS_LUT_SIZE];

#define PWM_BUFFER_SIZE     512
#define PWM_HALF_SIZE       256

typedef struct
{
    uint16_t CCR1;
    uint16_t CCR2;
    uint16_t CCR3;
} PWMFrame_t;

PWMFrame_t PWMBuffer[PWM_BUFFER_SIZE];



float DDS_GetSampleRate(void)
{
    uint32_t timclk = 216000000UL;

    uint32_t psc = TIM1->PSC;
    uint32_t arr = TIM1->ARR;

    if ((TIM1->CR1 & TIM_CR1_CMS) != 0)      // Center aligned
    {
        return (float)timclk /
               ((psc + 1.0f) * 2.0f * (arr + 1.0f));
    }
    else                                     // Up/Down
    {
        return (float)timclk /
               ((psc + 1.0f) * (arr + 1.0f));
    }
}

void DDS_GenerateTable(void)
{
    for(uint32_t i = 0; i < DDS_LUT_SIZE; i++)
    {
        float angle = 2.0f * (float)M_PI * (float)i / DDS_LUT_SIZE;

        SinTable[i] = (int16_t)(32767.0f * sinf(angle));
    }
}




void DDS_Init(void)
{
    DDS_GenerateTable();
    DDS_SetFrequency(60.0f);
    DDS_SetAmplitude(0.95f);
    DDS_SetPhase(0.0f);
    DDS.Direction = DDS_DIR_ABC;

    DDS.PhaseAccumulator = 0;

}

/*
void DDS_SetFrequency(float freq)
{
    DDS.Frequency = freq;

    DDS.PhaseIncrement =
        (uint32_t)
        (
            freq *
            4294967296.0f /
            65000.0f
        );
}
*/


void DDS_SetFrequency(float freq)
{
    DDS.Frequency = freq;

    float Fs = DDS_GetSampleRate();

    DDS.PhaseIncrement =
        (uint32_t)((freq * 4294967296.0f) / Fs);
}



void DDS_SetAmplitude(float amp)
{
    if(amp < 0.0f)
        amp = 0.0f;

    if(amp > 1.0f)
        amp = 1.0f;

    DDS.Amplitude = amp;

    DDS.AmplitudeScale = (int16_t)(32767.0f * amp);
}

void DDS_SetDirection(uint8_t dir)
{
    DDS.Direction=dir;
}

void DDS_SetPhase(float phase)
{
    while(phase < 0.0f)
        phase += 360.0f;

    while(phase >= 360.0f)
        phase -= 360.0f;

    DDS.PhaseShift = phase;

    DDS.PhaseOffset =
        (uint16_t)((phase * DDS_LUT_SIZE) / 360.0f);
}


void DDS_Update(void)
{
    DDS.PhaseAccumulator += DDS.PhaseIncrement;

    uint16_t index =
            DDS.PhaseAccumulator >> 21;

    uint16_t indexA;
    uint16_t indexB;
    uint16_t indexC;

    indexA = (index + DDS.PhaseOffset) & (DDS_LUT_SIZE - 1);

    if(DDS.Direction == DDS_DIR_ABC)
    {
        indexB = (indexA + DDS_PHASE120) & (DDS_LUT_SIZE - 1);
        indexC = (indexA + DDS_PHASE240) & (DDS_LUT_SIZE - 1);
    }
    else
    {
        indexB = (indexA + DDS_PHASE240) & (DDS_LUT_SIZE - 1);
        indexC = (indexA + DDS_PHASE120) & (DDS_LUT_SIZE - 1);
    }

    int32_t sA = SinTable[indexA];
    int32_t sB = SinTable[indexB];
    int32_t sC = SinTable[indexC];

    sA = (sA * DDS.AmplitudeScale) / 32767;
    sB = (sB * DDS.AmplitudeScale) / 32767;
    sC = (sC * DDS.AmplitudeScale) / 32767;

    uint16_t center = TIM1->ARR >> 1;

    TIM1->CCR1 = center + (sA * center) / 32767;
    TIM1->CCR2 = center + (sB * center) / 32767;
    TIM1->CCR3 = center + (sC * center) / 32767;
}


void DDS_FillBuffer(uint16_t start, uint16_t count)
{
    uint32_t phase = DDS.PhaseAccumulator;

    uint16_t center = TIM1->ARR >> 1;

    for(uint16_t i=0;i<count;i++)
    {
        phase += DDS.PhaseIncrement;

        uint16_t index = phase >> 21;

        uint16_t indexA = index;

        uint16_t indexB;

        uint16_t indexC;

        if(DDS.Direction == DDS_DIR_ABC)
        {
            indexB = (indexA + DDS_PHASE120) & (DDS_LUT_SIZE-1);
            indexC = (indexA + DDS_PHASE240) & (DDS_LUT_SIZE-1);
        }
        else
        {
            indexB = (indexA + DDS_PHASE240) & (DDS_LUT_SIZE-1);
            indexC = (indexA + DDS_PHASE120) & (DDS_LUT_SIZE-1);
        }

        int32_t sA = SinTable[indexA];
        int32_t sB = SinTable[indexB];
        int32_t sC = SinTable[indexC];

        sA = (sA * DDS.AmplitudeScale) / 32767;
        sB = (sB * DDS.AmplitudeScale) / 32767;
        sC = (sC * DDS.AmplitudeScale) / 32767;

        PWMBuffer[start+i].CCR1 = center + (sA * center)/32767;
        PWMBuffer[start+i].CCR2 = center + (sB * center)/32767;
        PWMBuffer[start+i].CCR3 = center + (sC * center)/32767;
    }

    DDS.PhaseAccumulator = phase;
}

