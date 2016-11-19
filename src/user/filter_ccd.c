#include "filter_ccd.h"

#define clamp(val,min,max) val < min ? min : val > max ? max : val
#define CCD_THRESH 100 //binary classifier ccd value [0 - 159]
#define WINDOWSIZE 10 //median filter windowsize

u32 medianCCD[128]; //stores medians
u32 schmittCCD[128]; //stores 0 or 1
u32 sumDiffCCD[128]; //stores sum prefix
u32 calibrateCCD[128]; //stores initial calibration offset

float getMedian(const int *a) {
    int arr[WINDOWSIZE] = {0};
    for (int k = 0; k < WINDOWSIZE; k++) { //copy into temporary array
        arr[k] = a[k];
    }
    int key, i, j;
    for (i = 2; i < WINDOWSIZE; i++) { //selection sort
        key = arr[i];
        j = i - 1;
        while (j > 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            --j;
        }
        arr[j + 1] = key;
    }
    return (WINDOWSIZE % 2 == 1 ? arr[WINDOWSIZE / 2] : (arr[WINDOWSIZE / 2 - 1] + arr[WINDOWSIZE / 2]) / 2);
}

void runSchmitt() {
    int k;
    for (k = 0; k < 128; k++) {
        schmittCCD[k] = (medianCCD[k] < CCD_THRESH) ? 1 : 158;
    }
}

void drawLine(int val, int isHorizontal, u16 color) {
    int k;
    if (isHorizontal) {
        for (k = 0; k < 159; k++)
            tft_put_pixel(k, val, color);
    } else {
        for (k = 0; k < 159; k++) {
            tft_put_pixel(val, k, color);
        }
    }
}

void calculateSumPrefix(int *leftCandidate, int *rightCandidate) {
    int k;
    for (k = 0; k < 128 - 1; k++) {
        sumDiffCCD[k] = schmittCCD[k] - schmittCCD[k + 1];
    }
    for (k = 0; k < 127; k++) { //negative value means close to left edge
        if (sumDiffCCD[k] < 0) {
            *leftCandidate = k;
            break;
        }
    }
    for (k = 126; k >= 0; k--) {
        if (sumDiffCCD[k] > 0) {
            *rightCandidate = k;
            break;
        }
    }
}

void calibrate_ccd() {
    int k;
    for (k = 0; k < 128; k++) {
        linear_ccd_buffer1[k] += calibrateCCD[k];
        linear_ccd_buffer1[k] = clamp(linear_ccd_buffer1[k], 0, 128);
    }
}

void runMedianFilter() {
    int curWindow[WINDOWSIZE] = {0};
    int indexOfOldest = 0;
    for (int k = 0; k < WINDOWSIZE; k++) {	//initialize the window
        curWindow[k] = linear_ccd_buffer1[k];
    }
    for (int j = 0; j <= WINDOWSIZE / 2; j++) {
        medianCCD[j] = getMedian(curWindow);
    }
    for (int k = WINDOWSIZE; k < 128; k++) {
        curWindow[indexOfOldest] = linear_ccd_buffer1[k];
        indexOfOldest++;
        indexOfOldest %= WINDOWSIZE;
        medianCCD[k - WINDOWSIZE / 2] = getMedian(curWindow);
    }
    for (int k = 128 - WINDOWSIZE; k < 128; k++) {
        medianCCD[k] = getMedian(curWindow);
    }
}
