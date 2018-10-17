/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#ifndef AD8302_H_INCLUDED
#define AD8302_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
   uint32_t valUref;
   uint32_t valUab;
   uint32_t valPhase;
} UADC;

typedef struct
{
    float R;
    float X;
} AD8302_RX;

UADC AD8302_ReadADC(void);
UADC AD8302_ReadADCOnce(void);

AD8302_RX AD8302_MeasureRX(void);

float AD8302_GetPhaseDegrees(UADC *pIn);
float AD8302_GetPhaseRadians(UADC *pIn);
float AD8302_GetRatioDb(UADC *pIn);
float AD8302_GetRatio(UADC *pIn);
float AD8302_CalcR(UADC *pIn);
float AD8302_CalcX(UADC *pIn);
float AD8302_CalcVSWR(float R, float X);

int AD8302_GetNoisePercent(void);
void AD8302_ResetNoiseCounter(void);
#ifdef __cplusplus
}
#endif

#endif //AD8302_H_INCLUDED