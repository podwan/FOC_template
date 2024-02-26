#ifndef __LOW_PASS_FILTER_H
#define __LOW_PASS_FILTER_H

#include "userMain.h"

typedef struct
{
    float timeConstant;
    float outputLast;
    unsigned long timeStamp;
} LowPassFilter;

void lowPassFilterInit(LowPassFilter *filter, const float timeConstant);
float lowPassFiltering(LowPassFilter *filter, float input);


#endif
