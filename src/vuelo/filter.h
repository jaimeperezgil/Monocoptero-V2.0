

#pragma once

#include "Arduino.h"

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
    float alpha;
} pt1Filter_t;

void pt1FilterInit(pt1Filter_t *filter, float f_cut, float dT);
void pt1FilterInitRC(pt1Filter_t *filter, float tau, float dT);
void pt1FilterSetTimeConstant(pt1Filter_t *filter, float tau);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float f_cut);
float pt1FilterGetLastOutput(pt1Filter_t *filter);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply3(pt1Filter_t *filter, float input, float dT);
float pt1FilterApply4(pt1Filter_t *filter, float input, float f_cut, float dt);
void pt1FilterReset(pt1Filter_t *filter, float input);

#define max_samp 10

typedef struct mediaFilter_s {
    float samp[max_samp];
    float tolerancia;
    int mediciones;
    int inicializado;
} mediaFilter_t;

void mediaFilter_init(mediaFilter_t *filtro, int mediciones);
void mediaFilter_setTolerace(mediaFilter_t *filtro, float tolerancia);
float mediaFilter_est(mediaFilter_t *filtro, float nueva);

/*
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

void biquadFilterInitNotch(biquadFilter_t *filter, uint32_t samplingIntervalUs, uint16_t filterFreq, uint16_t cutoffHz);
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs);
void biquadFilterInit(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float sample);
float biquadFilterReset(biquadFilter_t *filter, float value);
float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float filterGetNotchQ(float centerFrequencyHz, float cutoffFrequencyHz);
void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);*/
