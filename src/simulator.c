
#include "simulator.h"
#include "link_layer.h" 
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h> 

extern double FER_HEADER;  
extern double FER_DATA;   
extern unsigned int T_prop;  

static clock_t startTime, endTime;

void startTransferTimer() {
    startTime = clock();
}

double endTransferTimer() {
    endTime = clock();
    return (double)(endTime - startTime) / CLOCKS_PER_SEC;
}

int simulateError(double fer) {
    return ((double) rand() / RAND_MAX) < fer;
}

void simulatePropagationDelay() {
    usleep(T_prop); 
}

int llreadWithErrors(unsigned char* packet) {
    int packetSize = llread(packet);  

    if (simulateError(FER_HEADER)) {
        printf("Simulated error in packet header\n");
        return -1;  
    }

    if (simulateError(FER_DATA)) {
        printf("Simulated error in data field\n");
        return -1;  
    }

    return packetSize; 
}

