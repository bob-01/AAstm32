/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "stm32f1xx_hal.h"

#include "ad8302.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/*
    AD8302 is connected to measurement bridge as:

     Signal in  ----------------------
     from DDS     |                  |
                  |                  |
               RmeasAdd          RmeasAdd
                  |                  |
               [Antenna]           Rload
                  |-------INPB       |-------INPA
                Rmeas               Rmeas
                  |------ OFSB       |------ OFSA
                 ___      GND       ___
*/

//Connection of AD8302 to ADC:
//Vref - PC2
//Vmag - PC0
//Vpha - PC1

#define Rmeas 10.0
#define RmeasAdd 10.0
#define Rload 51.3
#define Rtotal (RmeasAdd + Rmeas + Rload)
#define Z0 50.0

#define ADC_NUM_MEASUREMENTS 21
#define ADC_NUM_MEASUREMENTS_REMAINING 11
#define ADC_DISPERSION_THRESHOLD 12 //in ADC units
#define ADC_NUM_ATTEMPTS 20

static const float PI = M_PI;
static uint32_t AD8302_Noise = 0;
static uint32_t AD8302_Measurements = 0;

//static AD8302_RX AD8302_FilterRX(UADC *pADC);
static UADC AD8302_FilterADC(UADC *pADC);

UADC AD8302_ReadADCOnce(void)
{
    UADC res = {0};

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,100);
    res.valPhase = (uint32_t) HAL_ADC_GetValue(&hadc1);        
    HAL_ADC_Stop(&hadc1);

    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2,100);
    res.valUab = (uint32_t) HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    res.valUref = 2234; //FIXME:

    return res;
}

int AD8302_GetNoisePercent(void)
{
    if (AD8302_Measurements == 0)
        return 0;
    return (int)((AD8302_Noise * 100) / AD8302_Measurements);
}

void AD8302_ResetNoiseCounter(void)
{
    AD8302_Noise = 0;
    AD8302_Measurements = 0;
}

static uint32_t _Dispersion(UADC *V)
{
    uint32_t i;
    uint32_t vmin = ULONG_MAX;
    uint32_t vmax = 0;
    uint32_t disp = 0;

    for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
    {
        if (V[i].valPhase < vmin)
            vmin = V[i].valPhase;
        if (V[i].valPhase > vmax)
            vmax = V[i].valPhase;
    }
    if ((vmax - vmin) > disp)
        disp = (vmax - vmin);

    vmin = ULONG_MAX;
    vmax = 0;
    for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
    {
        if (V[i].valUab < vmin)
            vmin = V[i].valUab;
        if (V[i].valUab > vmax)
            vmax = V[i].valUab;
    }
    if ((vmax - vmin) > disp)
        disp = (vmax - vmin);

    vmin = ULONG_MAX;
    vmax = 0;
    for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
    {
        if (V[i].valUref < vmin)
            vmin = V[i].valUref;
        if (V[i].valUref > vmax)
            vmax = V[i].valUref;
    }
    if ((vmax - vmin) > disp)
        disp = (vmax - vmin);
    return disp;
}

UADC AD8302_ReadADC(void)
{
    UADC results[ADC_NUM_MEASUREMENTS];
    UADC minDispResults[ADC_NUM_MEASUREMENTS];
    uint32_t i;
    uint32_t attempts = ADC_NUM_ATTEMPTS;
    uint32_t minDisp = ULONG_MAX;
    uint32_t current_disp = 0;
    memset(minDispResults, 0, sizeof(minDispResults));
    while(attempts)
    {
        for(i = 0; i < ADC_NUM_MEASUREMENTS; i++)
        {
            results[i] = AD8302_ReadADCOnce();
        }
        current_disp = _Dispersion(results);

        if (current_disp <= ADC_DISPERSION_THRESHOLD)
            break; //A good enough dispersion of measurements

        if (current_disp < minDisp)
        {//Remember the result of attempt with minimal dispersion
            memcpy(minDispResults, results, sizeof(results));
            minDisp = current_disp;
        }
        attempts--;
    }

    AD8302_Measurements++;
    if (attempts == 0)
    {//Use the stored result with minimal dispersion
        AD8302_Noise++;
        return AD8302_FilterADC(minDispResults);
    }
    return AD8302_FilterADC(results);
}

AD8302_RX AD8302_MeasureRX(void)
{
    UADC adcRes = AD8302_ReadADC();
    AD8302_RX rx;
    rx.R = AD8302_CalcR(&adcRes);
    rx.X = AD8302_CalcX(&adcRes);
    return rx;
}

static uint32_t _CalcMidU32(uint32_t* V)
{
    uint32_t counter;
    uint32_t i;
    uint32_t iVmin = 0;
    uint32_t iVmax = 0;
    uint32_t excluded[ADC_NUM_MEASUREMENTS];

    for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
    {
        excluded[i] = 0;
    }

    //Mark the most extremal ADC results as excluded, leaving only three of them finally.
    //This way is actually a little bit faster than usual sort/remove with our size of vector.
    counter = 0;
    while ((ADC_NUM_MEASUREMENTS - counter) > ADC_NUM_MEASUREMENTS_REMAINING)
    {
        uint32_t vmin = ULONG_MAX;
        uint32_t vmax = 0;
        for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
        {
            if (excluded[i]) //Bypass already excluded measurements
                continue;
            if (V[i] < vmin)
            {
                vmin = V[i];
                iVmin = i;
            }
            if (V[i] > vmax)
            {
                vmax = V[i];
                iVmax = i;
            }
        }
        excluded[iVmin] = 1;
        excluded[iVmax] = 1;
        if (iVmin != iVmax)
            counter += 2; //2 entries were excluded
        else
            break; //No need to exclude further? the rest of entries are equal by Z.
    }

    iVmin = 0; //Reusing this variable to calculate result
    counter = 0;
    for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
    {
        if (!excluded[i])
        {
            //todo: determine dispersion. If higher than some threshold - report error to redo measurements.
            iVmin += V[i];
            ++counter;
        }
    }
    iVmin /= counter; //todo: why not to use it undivided to improve resolution?
    return iVmin;
}

//Process vector of ADC results,
//filter out all measurement results except of 3 least extremal ones,
//and return average these 3 remaining measurements.
//This should remove all possible spurious outliers that influence the measurement precision.
static UADC AD8302_FilterADC(UADC *pADC)
{
    uint32_t i;
    uint32_t V[ADC_NUM_MEASUREMENTS];
    UADC resADC;

    assert_param(ADC_NUM_MEASUREMENTS & 1);  //Must be odd and >= 5
    assert_param(ADC_NUM_MEASUREMENTS >= 5);

    for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
        V[i] = pADC[i].valPhase;
    resADC.valPhase = _CalcMidU32(V);

    for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
        V[i] = pADC[i].valUab;
    resADC.valUab = _CalcMidU32(V);

    for (i = 0; i < ADC_NUM_MEASUREMENTS; i++)
        V[i] = pADC[i].valUref;
    resADC.valUref = _CalcMidU32(V);

    return resADC;
}

float AD8302_GetPhaseDegrees(UADC *in)
{
    if(in->valPhase > in->valUref)
    {
        in->valPhase = in->valUref;
    }
    float phi = PI - ((float)in->valPhase / (float)in->valUref) * PI;
    if (phi > PI / 2)
        phi = PI / 2;
    return (phi * 180.0) / PI;
}

float AD8302_GetPhaseRadians(UADC *in)
{
    if(in->valPhase > in->valUref)
    {
        in->valPhase = in->valUref;
    }
    float phi = PI - ((float)in->valPhase / (float)in->valUref) * PI;
    if (phi > PI / 2)
        phi = PI / 2;
    return phi;
}

float AD8302_GetRatioDb(UADC *in)
{
    if(in->valUab > in->valUref)
    {
        in->valUab = in->valUref;
    }
    float mid = ((float)in->valUref) / 2.0;
    float oneDb = mid / 30.0;
    float diff = (float)in->valUab - mid;
    float diffDb = diff / oneDb;
    return -diffDb;
}

float AD8302_GetRatio(UADC *in)
{
    float dB = AD8302_GetRatioDb(in);
    return powf(10.0, dB * 0.05);
}

float AD8302_CalcR(UADC *pIn)
{
    float _ratio, _phase, RR;
    assert_param(pIn);
    _ratio = AD8302_GetRatio(pIn);
    _phase = AD8302_GetPhaseRadians(pIn);
    RR = (cosf(_phase) * Rtotal * _ratio) - (Rmeas + RmeasAdd);
    if(RR < 0.0) //Sometimes this happens
        RR = 0.0;
    return RR;
}

float AD8302_CalcX(UADC *pIn)
{
    float _ratio, _phase, XX;
    assert_param(pIn);
    _ratio = AD8302_GetRatio(pIn);
    _phase = AD8302_GetPhaseRadians(pIn);
    XX = sinf(_phase) * Rtotal * _ratio;
    return XX;
}

//Calculate VSWR
float AD8302_CalcVSWR(float R, float X)
{
    float X2 = powf(X, 2);
    if(R < 5.0)
    {
        R = 0.0;
    }
    float ro = sqrtf((powf((R - Z0), 2) + X2) / (powf((R + Z0), 2) + X2));
    if(ro > .999)
    {
        ro = 0.999;
    }
    return (1.0 + ro) / (1.0 - ro);
}