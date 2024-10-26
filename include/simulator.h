
#ifndef PROTOCOL_UTILS_H
#define PROTOCOL_UTILS_H


void startTransferTimer();
double endTransferTimer();
int simulateError(double fer);
void simulatePropagationDelay();
int llreadWithErrors(unsigned char* packet);

#endif // PROTOCOL_UTILS_H
