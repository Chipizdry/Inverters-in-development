



#ifndef DDS3PHASE_H
#define DDS3PHASE_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define DDS_LUT_SIZE      2048
#define DDS_PWM_PERIOD    1660

#define DDS_DIR_ABC       0
#define DDS_DIR_ACB       1


#define DDS_PHASE120   (DDS_LUT_SIZE / 3)
#define DDS_PHASE240   (DDS_LUT_SIZE * 2 / 3)

typedef struct
{
    float Frequency;
    float Amplitude;
    float PhaseShift;

    uint8_t Direction;

    uint32_t PhaseAccumulator;
    uint32_t PhaseIncrement;

    uint16_t PhaseOffset;
    int16_t AmplitudeScale;

} DDS3PHASE_t;

extern DDS3PHASE_t DDS;

extern int16_t SinTable[DDS_LUT_SIZE];

void DDS_Init(void);

void DDS_GenerateTable(void);

void DDS_SetFrequency(float freq);

void DDS_SetAmplitude(float amp);

void DDS_SetDirection(uint8_t dir);

void DDS_SetPhase(float phase);

void DDS_Update(void);

#endif


