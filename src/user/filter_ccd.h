#ifndef _FILTER_CCD_H
#define _FILTER_CCD_H
#include "linear_ccd.h"
#include "lcd_blue.h"

extern u32 medianCCD[128];
extern u32 schmittCCD[128];
extern u32 sumDiffCCD[128];
extern u32 calibrateCCD[128];

float getMedian(const int *a);
void runSchmitt(void);
void drawLine(int val, int isHorizontal, u16 color);
void calculateSumPrefix(int *leftCandidate, int *rightCandidate);
void calibrate_ccd(void);
void runMedianFilter(void);

#endif
