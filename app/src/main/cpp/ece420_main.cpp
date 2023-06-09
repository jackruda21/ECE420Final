//
// Created by daran on 1/12/2017 to be used in ECE420 Sp17 for the first time.
// Modified by dwang49 on 1/1/2018 to adapt to Android 7.0 and Shield Tablet updates.
//

#include <jni.h>
#include "ece420_main.h"
#include "ece420_lib.h"
#include "kiss_fft/kiss_fft.h"
#include <cmath>
#include "Eigen/Core"
#include "Eigen/LU"

// Declare JNI function
extern "C" {
JNIEXPORT void JNICALL
Java_com_ece420_lab3_AudioActivity_getFftBuffer(JNIEnv *env, jclass, jobject bufferPtr);
}

extern "C" {
JNIEXPORT void JNICALL
Java_com_ece420_lab3_AudioActivity_getFftBufferClean(JNIEnv *env, jclass, jobject bufferPtr);
}

// FRAME_SIZE is 1024 and we zero-pad it to 2048 to do FFT
#define FRAME_SIZE 1024
#define ZP_FACTOR 2
#define FFT_SIZE (FRAME_SIZE * ZP_FACTOR)

//allows usage of cmath constants
#define _USE_MATH_DEFINES

// Variable to store final FFT output
float fftOut[FFT_SIZE] = {};
bool isWritingFft = false;

// how sensitive algo should be, will ignore below mean + SENSE * stDevDev, used for spectral gating
#define SENSE 1.5
// Variable to store noisy FFT output
float cleanOut[FFT_SIZE] = {};


void specSub(sample_buf *dataBuf) {
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

    // Window the frame
    for(int i =0; i<FRAME_SIZE;i++){
        bufferIn[i] = bufferIn[i] * 0.54 - .46*cos((2*M_PI*i)/(FRAME_SIZE-1));
    }

    // Zero pad
    float s_pad[FRAME_SIZE * ZP_FACTOR] = {};
    float magnitude[FRAME_SIZE] = {};

    // Fill in the zero_pad with data from the microphone
    for(int i=0; i<FRAME_SIZE; i++){
        s_pad[i] = bufferIn[i];
        magnitude[i] = 0;
    }

    // The following is to for creating a STFT
    int nfft = FRAME_SIZE * ZP_FACTOR;
    int is_inverse_fft = 0;
    kiss_fft_cpx cx_in[FRAME_SIZE * ZP_FACTOR] = {};   //input to be FFT'd
    kiss_fft_cpx cx_out[FRAME_SIZE * ZP_FACTOR] = {};  //FFT'd output

    kiss_fft_cfg cfg = kiss_fft_alloc( nfft ,is_inverse_fft,0,0);
    for(int i=0; i<nfft; i++){
        float window = 0.54 - (0.46 * cos(2*M_PI*i/(FRAME_SIZE-1)));  // hamming window
        cleanOut[i] = bufferIn[i] * window;
        // put kth sample in cx_in[k].r and cx_in[k].i
        cx_in[i].r = s_pad[i];
        cx_in[i].i = 0;
        //kiss_fft( cfg , cx_in , cx_out );
        // transformed. DC is in cx_out[0].r and cx_out[0].i

    }
    kiss_fft( cfg , cx_in , cx_out );
    free(cfg);
    // end of the STFT
    // implementation of spectral subtraction begins here
    // just copy and paste my proposal demo code translating from python to c++


    float maxVal = -FLT_MAX;
    float maxClean = -FLT_MAX;
    float mean = 0;
    //float floor = 1000;
    for(int i=0; i<nfft/2; i++){
        cleanOut[i] = 20*log10(sqrt((cx_out[i].r * cx_out[i].r) + (cx_out[i].i * cx_out[i].i)));
        //cleanOut[i] = abs((cx_out[i].r * cx_out[i].r) + (cx_out[i].i * cx_out[i].i));
        //magnitude[i] = abs((cx_out[i].r * cx_out[i].r) + (cx_out[i].i * cx_out[i].i)); // Compute magnitude power spectrum
        magnitude[i] = 20 * log10((cx_out[i].r * cx_out[i].r) + (cx_out[i].i * cx_out[i].i));
        mean += magnitude[i] / 2; // Computes noise floor estimate
        if(cleanOut[i]>maxClean){maxClean = cleanOut[i];}
        if(magnitude[i]>maxVal){maxVal = magnitude[i];} // Peak detection
    }
    mean = mean /(nfft/2);
    //float noise_max;
    for (int i=0; i<nfft/2; i++){
        //compute noise magnitude spectrum
        fftOut[i] = sqrt(fmax(magnitude[i] - mean,0));
        fftOut[i] = fmax(magnitude[i] - fftOut[i], 0);
    }
    // The following for normalizing between 0 and 1
    for(int i=0; i<nfft/2; i++){
        fftOut[i] = (fftOut[i]/maxVal);
        cleanOut[i] = cleanOut[i]/maxClean;
    }

    // thread-safe
    isWritingFft = true;
    // Currently set everything to 0 or 1 so the spectrogram will just be blue and red stripped
    /*for ( int i= 0; i < FRAME_SIZE; i++) {
        fftOut[i] = (i/20)%2;
    }*/

    // ********************* END YOUR CODE HERE ************************* //
    // Flip the flag so that the JNI thread will update the buffer
    isWritingFft = false;

    gettimeofday(&end, NULL);
    LOGD("Time delay: %ld us",  ((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec)));
}


// First frame flag so we know when to actually create threshhold
int first_frame = 10;
float mean = 0;
float stDev = 0;

void specGate(sample_buf *dataBuf) {
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

    // ********************* START YOUR CODE HERE *********************** //

    //window the frame

    for(int i =0; i<FRAME_SIZE;i++){
        bufferIn[i] = bufferIn[i] * .54 - .46*cos((2*M_PI*i)/(FRAME_SIZE-1));
    }
    //zero pad
    float s_pad[FRAME_SIZE * ZP_FACTOR] = {};
    float magnitude[FRAME_SIZE] = {};

    // Fill in the zero_pad with data from the microphone
    for(int i=0; i<FRAME_SIZE; i++){
        s_pad[i] = bufferIn[i];
        magnitude[i] = 0;
    }

    // The following is to for creating a STFT
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
        cleanOut[i] = 20*log10(sqrt(cx_out[i].r * cx_out[i].r + cx_out[i].i * cx_out[i].i));
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
        stDev = sqrt(stDev/(nfft/2));
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
        cleanOut[i] = cleanOut[i]/max;
    }


/*
    float max = -FLT_MAX;
    for(int i=0; i<nfft/2; i++){
        //fftOut[i] = 20*log10(sqrt(cx_out[i].r * cx_out[i].r + cx_out[i].i * cx_out[i].i));
        magnitude[i] = abs(cx_out[i].r * cx_out[i].r + cx_out[i].i * cx_out[i].i); // Compute magnitude power spectrum
        mean += magnitude[i] / 2; // Computes noise floor estimate
        if(magnitude[i]>maxVal){maxVal = magnitude[i];} // Peak detection
    }
    mean /= (nfft/2);
    //float noise_max;
    for (int i=0; i<nfft/2; i++){
        //compute noise magnitude spectrum
        fftOut[i] = sqrt(fmax(magnitude[i] - floor,0));
        fftOut[i] = fmax(magnitude[i] - fftOut[i], 0);
    }
*/
/*
    for(int i=0; i<nfft/2; i++){
        fftOut[i] = fftOut[i]/max;
    }
*/



    //audio playback

    //ifft
    is_inverse_fft = 1;
    cfg = kiss_fft_alloc( nfft ,is_inverse_fft,0,0);
    kiss_fft( cfg , cx_out , cx_in );
    free(cfg);

    uint16_t outmag = 0;
    //set output buffer
    for(int i=0; i<FRAME_SIZE; i++){
        outmag = (uint16_t)sqrt(cx_out[i].r * cx_out[i].r + cx_out[i].i * cx_out[i].i);
        dataBuf->buf_[2*i] = (uint8_t)outmag;
        dataBuf->buf_[2*i+1] = (uint8_t)(outmag>>8);
    }

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

//number of frames we need to store to train filter
#define FRAMES_STORED 20

//file-scope variables for wiener filtering
int frame_count_rec = 0;
int frame_count_start = 0;
float clean_frames[FRAME_SIZE * FRAMES_STORED] = {};
float noise_frames[FRAME_SIZE * FRAMES_STORED] = {};

float matrixA[10000][10] = {};
float hFilter[10] = {};
float vector[10000] = {};

//flag to denote when filter has been created
int filter_flag = 0;

//TEST VARIABLES, REMOVE
int recording_clean = 1;
int wiener_start = 0;

//variable to hold our filter
kiss_fft_cpx wiener_filter[FRAME_SIZE * ZP_FACTOR] = {};

//linalg typedefs
typedef Eigen::Matrix<float, 10000,10> MatrixBig;
typedef Eigen::Matrix<float, 10,1>     Matrix10;
typedef Eigen::Matrix<float, 10000,1>  MatrixLong;

//Matrices for linalg ops
//MatrixBig  A;
//Matrix10   h;
//MatrixLong x;

//(a+bi)(c+di) = ac - bd + (bc+ad)i
kiss_fft_cpx cpx_mult(kiss_fft_cpx a, kiss_fft_cpx b){
    kiss_fft_cpx result;
    result.r = a.r*b.r - a.i*b.i;
    result.i = a.i*b.r + a.r*b.i;
    return result;
}

void wiener(sample_buf *dataBuf) {
    isWritingFft = false;
    // Keep in mind, we only have 20ms to process each buffer!

    // Data is encoded in signed PCM-16, little-endian, mono channel
    float bufferIn[FRAME_SIZE];
    for (int i = 0; i < FRAME_SIZE; i++) {
        int16_t val = ((uint16_t) dataBuf->buf_[2 * i]) | (((uint16_t) dataBuf->buf_[2 * i + 1]) << 8);
        bufferIn[i] = (float) val;
    }

    for(int i =0; i<FRAME_SIZE;i++){
        bufferIn[i] = bufferIn[i] * .54 - .46*cos((2*M_PI*i)/(FRAME_SIZE-1));
    }


    //store 10 frames of clean recorded audio data when rec. button is pressed
    if(recording_clean){
        LOGD("rec_count: %d", frame_count_rec);
        if(frame_count_rec < FRAMES_STORED){
            for(int i=0; i<FRAME_SIZE; i++){
                clean_frames[frame_count_rec * FRAME_SIZE + i] = bufferIn[i];
            }
            frame_count_rec++;
        }
        if(frame_count_rec == FRAMES_STORED){
            recording_clean = 0;
            wiener_start = 1;
        }
    }

    //store 10 frames of noisy audio data once start is pressed
    if(wiener_start){
        //record 1st 10 frames of data
        LOGD("start_count: %d", frame_count_start);
        if(frame_count_start < FRAMES_STORED){
            for(int i=0; i<FRAME_SIZE; i++){
                noise_frames[frame_count_start * FRAME_SIZE + i] = bufferIn[i];
            }
            frame_count_start++;
            filter_flag = 0;
        }
        if(frame_count_start == FRAMES_STORED){
            filter_flag = 1;
            wiener_start = 0;
        }
    }

        //after recording those frames, process the filter

    if(filter_flag){
        LOGD("Filtering...");
        //process recorded data and create filter
        // The following were too big to be allocated on the stack so are globals instead
        Eigen::MatrixXf A(10000,10);
        Eigen::MatrixXf h(10,1);
        Eigen::MatrixXf x(10000,1);
        for(int i = 0; i<10000; i++){
            for(int j = 0; j<10; j++){
                A(i,j) = noise_frames[i+j];
            }
            x(i,0) = clean_frames[i];
        }

        //h is our filter coefficients
        h = (A.transpose() * A).inverse() * A.transpose() * x;
        //A = A.inverse();

        //fft the filter
        int nfft_filter = FRAME_SIZE * ZP_FACTOR;
        int is_inverse_fft_filter = 0;
        kiss_fft_cpx cx_in_filter[10] = {};   //input to be FFT'd

        kiss_fft_cfg cfg = kiss_fft_alloc( nfft_filter ,is_inverse_fft_filter,0,0);

        for(int i=0; i<10; i++){
            // put kth sample in cx_in[k].r and cx_in[k].i
            cx_in_filter[i].r = h(i,0);
            cx_in_filter[i].i = 0;
        }
        //perform fft, only required for output spectrogram
        kiss_fft( cfg , cx_in_filter , wiener_filter );
        //our filter is now in the file-scope variable wiener_filter to be used for processing
        free(cfg);
        filter_flag = 0;
    }

    //zero pad
    float s_pad[FRAME_SIZE * ZP_FACTOR] = {};

    for(int i=0; i<FRAME_SIZE; i++){
        s_pad[i] = bufferIn[i];
    }

    //set up fft stuff
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
    //perform fft, only required for output spectrogram
    kiss_fft( cfg , cx_in , cx_out );
    free(cfg);

    //filter input data

    float max = -FLT_MAX;
    float cleanMax = -FLT_MAX;
    kiss_fft_cpx temp;
    for (int i = 0; i < nfft/2; i++) {
        temp = cpx_mult(cx_out[i], wiener_filter[i]);
        fftOut[i] = 20*log10(sqrt(temp.r * temp.r + temp.i * temp.i));
        cleanOut[i] = 20*log10(sqrt(cx_out[i].r * cx_out[i].r + cx_out[i].i * cx_out[i].i));
        if(fftOut[i]>max){
            max = fftOut[i];
        }
        if(cleanOut[i]>cleanMax){
            cleanMax = cleanOut[i];
        }
    }

    //normalize values
    for (int i = 0; i < nfft/2; i++) {
        fftOut[i] = fftOut[i]/max;
        cleanOut[i] = cleanOut[i]/cleanMax;
    }
}

// http://stackoverflow.com/questions/34168791/ndk-work-with-floatbuffer-as-parameter
extern "C" JNIEXPORT void JNICALL
Java_com_ece420_lab3_AudioActivity_getFftBuffer(JNIEnv *env, jclass, jobject bufferPtr) {
    jfloat *buffer = (jfloat *) env->GetDirectBufferAddress(bufferPtr);

    // thread-safe, kinda
    while (isWritingFft) {}
    // We will only fetch up to FRAME_SIZE data in fftOut[] to draw on to the screen
    for (int i = 0; i < FRAME_SIZE; i++) {
        buffer[i] = fftOut[i];
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_ece420_lab3_AudioActivity_getFftBufferClean(JNIEnv *env, jclass, jobject bufferPtr) {
    jfloat *buffer = (jfloat *) env->GetDirectBufferAddress(bufferPtr);

    // thread-safe, kinda
    while (isWritingFft) {}
    // We will only fetch up to FRAME_SIZE data in fftOut[] to draw on to the screen
    for (int i = 0; i < FRAME_SIZE; i++) {
        buffer[i] = cleanOut[i];
        //buffer[i] = 1;
    }
}
