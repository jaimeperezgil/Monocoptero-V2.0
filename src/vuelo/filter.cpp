
#include "filter.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define M_PIf       3.14159265358979323846f

// PT1 Low Pass filter

static float pt1ComputeRC(const float f_cut)
{
    return 1.0f / (2.0f * M_PIf * f_cut);
}

// f_cut = cutoff frequency
void pt1FilterInitRC(pt1Filter_t *filter, float tau, float dT)
{
    filter->state = 0.0f;
    filter->RC = tau;
    filter->dT = dT;
    filter->alpha = filter->dT / (filter->RC + filter->dT);
}

void pt1FilterInit(pt1Filter_t *filter, float f_cut, float dT)
{
    pt1FilterInitRC(filter, pt1ComputeRC(f_cut), dT);
}

void pt1FilterSetTimeConstant(pt1Filter_t *filter, float tau) {
    filter->RC = tau;
}

float pt1FilterGetLastOutput(pt1Filter_t *filter) {
    return filter->state;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float f_cut)
{
    filter->RC = pt1ComputeRC(f_cut);
    filter->alpha = filter->dT / (filter->RC + filter->dT);
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->alpha * (input - filter->state);
    return filter->state;
}

float pt1FilterApply3(pt1Filter_t *filter, float input, float dT)
{
    filter->dT = dT;
    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    return filter->state;
}

float pt1FilterApply4(pt1Filter_t *filter, float input, float f_cut, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC) {
        filter->RC = pt1ComputeRC(f_cut);
    }

    filter->dT = dT;    // cache latest dT for possible use in pt1FilterApply
    filter->alpha = filter->dT / (filter->RC + filter->dT);
    filter->state = filter->state + filter->alpha * (input - filter->state);
    return filter->state;
}

void pt1FilterReset(pt1Filter_t *filter, float input)
{
    filter->state = input;
}


void mediaFilter_init(mediaFilter_t *filtro, int mediciones){
    filtro->mediciones=mediciones;
    filtro->tolerancia=2;
    filtro->samp[1]=10;
}

void mediaFilter_setTolerace(mediaFilter_t *filtro, float tolerancia){
    filtro->tolerancia=tolerancia;
}

float mediaFilter_est(mediaFilter_t *filtro, float nueva){
    double valAcc=0;
    for(int i=0;i<filtro->mediciones;i++){
        valAcc+=filtro->samp[i];
    }
    double avr=valAcc/filtro->mediciones;
    
    if((avr+filtro->tolerancia>nueva && avr-filtro->tolerancia<nueva) || filtro->inicializado<filtro->mediciones+10){
        filtro->inicializado++;
        for(int i=1;i<filtro->mediciones;i++){
            filtro->samp[i-1]=filtro->samp[i];
        }
        filtro->samp[filtro->mediciones-1]=nueva;
        return nueva;
    }else{
        return avr;
    }
}







void biquadFilterInitNotch(biquadFilter_t *filter, uint32_t samplingIntervalUs, uint16_t filterFreq, uint16_t cutoffHz)
{
    float Q = filterGetNotchQ(filterFreq, cutoffHz);
    biquadFilterInit(filter, filterFreq, samplingIntervalUs, Q, FILTER_NOTCH);
}

// sets up a biquad Filter
/*void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs)
{
    biquadFilterInit(filter, filterFreq, samplingIntervalUs, BIQUAD_Q, FILTER_LPF);
}*/


static void biquadFilterSetupPassthrough(biquadFilter_t *filter)
{
    // By default set as passthrough
    filter->b0 = 1.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;
}

void biquadFilterInit(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs, float Q, biquadFilterType_e filterType)
{
    // Check for Nyquist frequency and if it's not possible to initialize filter as requested - set to no filtering at all
    if (filterFreq < (1000000 / samplingIntervalUs / 2)) {
        // setup variables
        const float sampleRate = 1.0f / ((float)samplingIntervalUs * 0.000001f);
        const float omega = 2.0f * M_PIf * ((float)filterFreq) / sampleRate;
        const float sn = sin(omega);
        const float cs = cos(omega);
        const float alpha = sn / (2 * Q);

        float b0, b1, b2;
        switch (filterType) {
            case FILTER_LPF:
                b0 = (1 - cs) / 2;
                b1 = 1 - cs;
                b2 = (1 - cs) / 2;
                break;
            case FILTER_NOTCH:
                b0 = 1;
                b1 = -2 * cs;
                b2 = 1;
                break;
            default:
                biquadFilterSetupPassthrough(filter);
                return;
        }
        const float a0 =  1 + alpha;
        const float a1 = -2 * cs;
        const float a2 =  1 - alpha;

        // precompute the coefficients
        filter->b0 = b0 / a0;
        filter->b1 = b1 / a0;
        filter->b2 = b2 / a0;
        filter->a1 = a1 / a0;
        filter->a2 = a2 / a0;
    } else {
        biquadFilterSetupPassthrough(filter);
    }

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    // shift x1 to x2, input to x1 
    filter->x2 = filter->x1;
    filter->x1 = input;

    // shift y1 to y2, result to y1 
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
}

float biquadFilterReset(biquadFilter_t *filter, float value)
{
    filter->x1 = value - (value * filter->b0);
    filter->x2 = (filter->b2 - filter->a2) * value;
    return value;
}

void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType)
{
    // backup state
    float x1 = filter->x1;
    float x2 = filter->x2;
    float y1 = filter->y1;
    float y2 = filter->y2;

    biquadFilterInit(filter, filterFreq, refreshRate, Q, filterType);

    // restore state
    filter->x1 = x1;
    filter->x2 = x2;
    filter->y1 = y1;
    filter->y2 = y2;
}
