
#include "simulator.h"
#include "link_layer.h"  // Assuming link_layer.h contains llread definition
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>  // For usleep()

// Configuration for FER and propagation delay
extern double FER_HEADER;  // Defined in application layer
extern double FER_DATA;    // Defined in application layer
extern unsigned int T_prop;  // Defined in application layer

static clock_t startTime, endTime;

// Start the transfer timer
void startTransferTimer() {
    startTime = clock();
}

// End the transfer timer and return elapsed time in seconds
double endTransferTimer() {
    endTime = clock();
    return (double)(endTime - startTime) / CLOCKS_PER_SEC;
}

// Simulates an error with probability fer
int simulateError(double fer) {
    return ((double) rand() / RAND_MAX) < fer;
}

// Simulates propagation delay
void simulatePropagationDelay() {
    usleep(T_prop);  // Introduce a delay in microseconds
}

// Simulates errors in packet reception
int llreadWithErrors(unsigned char* packet) {
    int packetSize = llread(packet);  // Normal llread

    // Simulate random error in header
    if (simulateError(FER_HEADER)) {
        printf("Simulated error in packet header\n");
        return -1;  // Simulate a failed packet reception due to header error
    }

    // Simulate random error in data field
    if (simulateError(FER_DATA)) {
        printf("Simulated error in data field\n");
        return -1;  // Simulate a failed packet reception due to data error
    }

    return packetSize;  // Return the actual packet size if no error occurs
}
