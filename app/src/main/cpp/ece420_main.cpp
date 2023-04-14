//
// Created by daran on 1/12/2017 to be used in ECE420 Sp17 for the first time.
// Modified by dwang49 on 1/1/2018 to adapt to Android 7.0 and Shield Tablet updates.
//

#include <jni.h>
#include "ece420_main.h"
#include "ece420_lib.h"
#include "kiss_fft/kiss_fft.h"
#include <cmath>

// Declare JNI function
extern "C" {
JNIEXPORT void JNICALL
Java_com_ece420_lab3_MainActivity_getFftBuffer(JNIEnv *env, jclass, jobject bufferPtr);
}

// FRAME_SIZE is 1024 and we zero-pad it to 2048 to do FFT
#define FRAME_SIZE 1024
#define ZP_FACTOR 2
#define FFT_SIZE (FRAME_SIZE * ZP_FACTOR)

//allows usage of cmath constants
#define _USE_MATH_DEFINES

// how sensitive algo should be, will ignore below mean + SENSE * stDevDev
#define SENSE .25

// Variable to store final FFT output
float fftOut[FFT_SIZE] = {};
bool isWritingFft = false;

// First frame flag so we know when to actually create threshhold
int first_frame = 10;
float mean = 0;
float stDev = 0;

void ece420ProcessFrame(sample_buf *dataBuf) {
    isWritingFft = false;
    // Keep in mind, we only have 20ms to process each buffer!
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);

    // Data is encoded in signed PCM-16, little-endian, mono channel
    float bufferIn[FRAME_SIZE];
    for (int i = 0; i < FRAME_SIZE; i++) {
        int16_t val = ((uint16_t) dataBuf->buf_[2 * i]) | (((uint16_t) dataBuf->buf_[2 * i + 1]) << 8);
        bufferIn[i] = (float) val;
    }

    // Spectrogram is just a fancy word for short time fourier transform
    // 1. Apply hamming window to the entire FRAME_SIZE
    // 2. Zero padding to FFT_SIZE = FRAME_SIZE * ZP_FACTOR
    // 3. Apply fft with KISS_FFT engine
    // 4. Scale fftOut[] to between 0 and 1 with log() and linear scaling
    // NOTE: This code block is a suggestion to get you started. You will have to
    // add/change code outside this block to implement FFT buffer overlapping (extra credit part).
    // Keep all of your code changes within java/MainActivity and cpp/ece420_*
    // ********************* START YOUR CODE HERE *********************** //

    //window the frame
    for(int i =0; i<FRAME_SIZE;i++){
        bufferIn[i] = bufferIn[i] * .54 - .46*cos((2*M_PI*i)/(FRAME_SIZE-1));
    }

    //zero pad
    float s_pad[FRAME_SIZE * ZP_FACTOR] = {};

    for(int i=0; i<FRAME_SIZE; i++){
        s_pad[i] = bufferIn[i];
    }

    int nfft = FRAME_SIZE * ZP_FACTOR;
    int is_inverse_fft = 0;
    kiss_fft_cpx cx_in[FRAME_SIZE * ZP_FACTOR] = {};   //input to be FFT'd
    kiss_fft_cpx cx_out[FRAME_SIZE * ZP_FACTOR] = {};  //FFT'd output

    kiss_fft_cfg cfg = kiss_fft_alloc( nfft ,is_inverse_fft,0,0);
    for(int i=0; i<nfft; i++){

        // put kth sample in cx_in[k].r and cx_in[k].i
        cx_in[i].r = s_pad[i];
        cx_in[i].i = 0;
        //kiss_fft( cfg , cx_in , cx_out );
        // transformed. DC is in cx_out[0].r and cx_out[0].i

    }
    kiss_fft( cfg , cx_in , cx_out );
    free(cfg);

    float min = FLT_MAX;
    float max = -FLT_MAX;
    //make output dB scale and find min and max
    for(int i=0; i<nfft/2; i++){
        fftOut[i] = 20*log10(sqrt(cx_out[i].r * cx_out[i].r + cx_out[i].i * cx_out[i].i));
        if(fftOut[i]<min){
            min = fftOut[i];
        }
        if(fftOut[i]>max){
            max = fftOut[i];
        }
    }

    //cx_out is our FFT'd signal
    //calculate mean and standard deviation
    if(first_frame){
        float sum = 0.0;
        for (int i = 0; i < nfft/2; i++) {
            sum += fftOut[i];
        }
        mean = sum/(nfft/2);
        for (int i = 0; i < nfft/2; i++) {
            stDev += pow(fftOut[i] - mean, 2);
        }
        //stDev = sqrt(stDev);
        if(first_frame > 0){
            first_frame --;
        }
    }
    LOGD("Mean: %f, Max: %f, Min: %f, stDev: %f", mean, max, min, stDev);


    // Smoothing filter ... ignore for now

    //apply masking
    float thresh = mean + SENSE * stDev;
    for (int i = 0; i < nfft/2; i++) {
        if(fftOut[i] < thresh){
            fftOut[i] = 0; //(min < 0 ? 0 : min); //for visualization, just set it = to 0
            cx_out[i].i = 0;
            cx_out[i].r = min;
        }
        fftOut[i] = fftOut[i]/max;
    }


/*
    float max = -FLT_MAX;
    for(int i=0; i<nfft/2; i++){
        fftOut[i] = log10(cx_out[i].r * cx_out[i].r + cx_out[i].i * cx_out[i].i);
        if(fftOut[i]>max){
            max = fftOut[i];
        }
    }
*/
/*
    for(int i=0; i<nfft/2; i++){
        fftOut[i] = fftOut[i]/max;
    }

*/

    // thread-safe
    isWritingFft = true;
    // Currently set everything to 0 or 1 so the spectrogram will just be blue and red stripped
    /*for ( int i= 0; i < FRAME_SIZE; i++) {
        fftOut[i] = (i/20)%2;
    }*/

    /*
    Given a buffer size N and a sampling rate Fs, how much time do you have to complete your
    processing before the next buffer comes in?

    N * 1/Fs because we are getting Fs samples/sec, so each sample takes 1/Fs time to get.
    Since there are N samples per buffer, the buffer will fill up in N * 1/Fs seconds.
    */
    // ********************* END YOUR CODE HERE ************************* //
    // Flip the flag so that the JNI thread will update the buffer
    isWritingFft = false;

    gettimeofday(&end, NULL);
    LOGD("Time delay: %ld us",  ((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec)));
}


// http://stackoverflow.com/questions/34168791/ndk-work-with-floatbuffer-as-parameter
JNIEXPORT void JNICALL
Java_com_ece420_lab3_MainActivity_getFftBuffer(JNIEnv *env, jclass, jobject bufferPtr) {
    jfloat *buffer = (jfloat *) env->GetDirectBufferAddress(bufferPtr);
    // thread-safe, kinda
    while (isWritingFft) {}
    // We will only fetch up to FRAME_SIZE data in fftOut[] to draw on to the screen
    for (int i = 0; i < FRAME_SIZE; i++) {
        buffer[i] = fftOut[i];
    }
}
