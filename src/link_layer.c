
#include "link_layer.h"
#include "serial_port.h"

#define _POSIX_SOURCE 1 

int alarmEnabled = FALSE;
int firstFrame = TRUE;
int alarmCount = 0;
int max_retransmissions = 0;
int timeout = 0;

unsigned char send = 0;
unsigned char receive = 1;

extern int fd;

typedef enum
{
   START,
   FLAG_RCV,
   A_RCV,
   C_RCV,
   BCC1_OK,
   BCC2_OK,
   STOP_,
   DETECTED_ESC,
   PROCESSING
} LLState;

typedef struct {
    unsigned char expected_addr;
    unsigned char send_ctrl;
    unsigned char recv_ctrl;
    unsigned char response_ctrl;
} RoleParams;

typedef struct {
    int totalTransmissions;    
    int successfulTransmissions; 
    int failedTransmissions;   
    int totalBytesSent;         
} LinkStatistics;

LinkStatistics stats = {0};

#define T_SECONDS 3
#define BUF_SIZE 5 

#define FLAG    0x7E 
#define A1      0x03 
#define A2      0x01 
#define ESC     0x7D 

#define SET     0X03
#define UA      0x07
#define RR0     0xAA 
#define RR1     0xAB 
#define REJ0    0x54 
#define REJ1    0x55 
#define DISC    0x0B 
#define BCC1(a,c) ((a)^(c)) 

#define INFO0    0x00 
#define INFO1    0x80 


#define RR(Nr) ((Nr == 0) ? RR0 : RR1)
#define REJ(Nr) ((Nr == 0) ? REJ0 : REJ1)
#define I(Ns)  ((Ns == 0) ? INFO0 : INFO1)


void alarmHandler(int signal);
void initAlarm();
int sendSupFrame(int fd, unsigned char addr, unsigned char ctrl);
LLState SetUaStateMachine(int fd, unsigned char expectedAddr, unsigned char expectedCtrl, unsigned char expectedBCC);
unsigned char calculateBCC2(const unsigned char *buf, int bufSize);
void byteStuffingTechnique(unsigned char **frame, int *frameSize, unsigned char byte, int *j);
unsigned char infoFrameStateMachine(int fd);



void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Waiting... #%d\n", alarmCount);
}

void initAlarm() {
    struct sigaction act = { 0 };
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
}

int sendSupFrame(int fd, unsigned char addr, unsigned char ctrl) {
    unsigned char frame[BUF_SIZE] = {FLAG, addr, ctrl, addr ^ ctrl, FLAG};
    int bytes = write(fd, frame, BUF_SIZE);
    return bytes;
}


LLState SetUaStateMachine (int fd, unsigned char expectedAddr, unsigned char expectedCtrl, unsigned char expectedBCC) { 
    unsigned char buf[BUF_SIZE] = {0};
    LLState state = START;
    while (state != STOP_){ 
        int bytes = read(fd, buf, BUF_SIZE);
        if (bytes < 0) {
            perror("Error reading from serial port");
            return START;
        }

        for (int i = 0; i < bytes; i++) {
            unsigned char byte = buf[i];
            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    break;

                case FLAG_RCV:
                    if (byte == FLAG) {
                        state = START;
                    } else if (byte == expectedAddr) {
                        state = A_RCV;
                    }
                    break;

                case A_RCV:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else if (byte == expectedCtrl) {
                        state = C_RCV;
                    } else {
                        state = START;
                    }
                    break;

                case C_RCV:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else if (byte == expectedBCC) {
                        state = BCC1_OK;
                    } else {
                        state = START;
                    }
                    break;

                case BCC1_OK:
                    if (byte == FLAG) {
                        state = STOP_;
                    } else {
                        state = START;
                    }
                    break;

                default:
                    state = START;
                    break;
            }
        }
    }
    return state;
}

int llopen(LinkLayer connectionParameters) {
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        return -1;
    }
    
    max_retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    switch (connectionParameters.role) {
        case LlTx: {  

            while (alarmCount < max_retransmissions) {
                initAlarm();
                
                if (alarmEnabled == FALSE) {
                    sendSupFrame(fd, A1, SET);
                    alarm(timeout); 
                    alarmEnabled = TRUE;
                }

                LLState state = SetUaStateMachine(fd, A1, UA, BCC1(A1, UA));
                
                if (state == STOP_) { 
                    alarm(0); 
                    return 1; 
                }
            }
            
            printf("Maximum number of timeouts exceeded. Communication could not be established.\n");
            return -1;
        }

        case LlRx: { 

            LLState state = SetUaStateMachine(fd, A1, SET, BCC1(A1, SET));
            
            if (state == STOP_) { 
                sendSupFrame(fd, A1, UA); 
                return 1; 
            }
            
            return -1;
        }
    }
    return -1; 
}


unsigned char calculateBCC2(const unsigned char *buf, int bufSize) {
    unsigned char BCC2 = buf[0];
    for (int i = 1; i < bufSize; i++) {
        BCC2 ^= buf[i];
    }
    return BCC2;
}

void byteStuffingTechnique(unsigned char **frame, int *size, unsigned char byte, int *index) {
    if (byte == FLAG || byte == ESC) { 
        *frame = realloc(*frame, *size + 1);  
        (*frame)[*index] = ESC;
        (*frame)[(*index) + 1] = byte ^ 0x20; 
        *index += 2;
        *size += 1; 
    } else {
        (*frame)[*index] = byte;
        (*index)++;
    }
}



unsigned char infoFrameStateMachine(int fd) {
    unsigned char infoFrame = 0;
    LLState state = START;

    unsigned char buf[BUF_SIZE] = {0};
    while (state != STOP_ && !alarmEnabled) {
        int bytesRead = read(fd, buf, BUF_SIZE);
        
        if (bytesRead <= 0) {
            continue;  
        }

        for (int i = 0; i < bytesRead; i++) {
            unsigned char byte = buf[i];
            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;  
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A1) {
                        state = A_RCV; 
                    } else if (byte != FLAG) {
                        state = START;
                    }
                    break;
                case A_RCV:
                    if (byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1 || byte == DISC) {
                        infoFrame = byte;  
                        state = C_RCV;  
                    } else if (byte == FLAG) {
                        state = FLAG_RCV; 
                    } else {
                        state = START;
                    }
                    break;
                case C_RCV:
                    if (byte == BCC1(A1, infoFrame)) {
                        state = BCC1_OK;
                    } else if (byte == FLAG) {
                        state = FLAG_RCV; 
                    } else {
                        state = START; 
                    }
                    break;
                case BCC1_OK:
                    if (byte == FLAG) {
                        state = STOP_;
                    } else {
                        state = START; 
                    }
                    break;
                default:
                    state = START;
                    break;
            }
        }
    }
    return infoFrame; 
}

int llwrite(const unsigned char *buf, int bufSize) {
    int totalSize = 6 + bufSize; 
    unsigned char *frame = (unsigned char *) malloc(totalSize);

    frame[0] = FLAG;                
    frame[1] = A1;                
    frame[2] = I(send); 
    frame[3] = frame[1] ^ frame[2];  

    memcpy(frame + 4, buf, bufSize);

    int j = 4;
    for (int i = 0; i < bufSize; i++) {
        byteStuffingTechnique(&frame, &totalSize, buf[i], &j);
    }

    unsigned char BCC2 = calculateBCC2(buf, bufSize);
    byteStuffingTechnique(&frame, &totalSize, BCC2, &j);
    
    frame[j++] = FLAG;

    int transmission = 0;
    int check_rej = 0, check_rr = 0;

    while (transmission < max_retransmissions) { 
        alarmEnabled = FALSE; 
        alarm(timeout);       
        check_rej = 0;
        check_rr = 0;

        while (!alarmEnabled && !check_rej && !check_rr) {
            write(fd, frame, j);

            stats.totalBytesSent += j;

            unsigned char res = infoFrameStateMachine(fd); 

            if (!res) {
                continue;
            } else if (res == REJ(0) || res == REJ(1)) { 
                check_rej = 1;
            } else if (res == RR(0) || res == RR(1)) { 
                check_rr = 1;
                send = (send + 1) % 2; 
            } else {
                continue;
            }
        }

        if (check_rr) {
            stats.totalBytesSent += totalSize;
            alarm(0); 
            alarmCount = 0; 
            break;
        }

        printf("Transmission failed, attempt %d.\n", transmission + 1);
        transmission++;
    }

    free(frame);

    if (check_rr) {
        return totalSize;
    } else {
        printf("Max transmissions reached. Closing link layer.\n");
        llclose(fd);
        return -1;
    }
}


int processingData(unsigned char *packet, unsigned char byte, int *i, unsigned char infoFrame, LLState *state) {
    unsigned char bcc2 = packet[*i - 1];
    (*i)--; 

    unsigned char calculatedBCC2 = calculateBCC2(packet, *i);
    if (bcc2 == calculatedBCC2) {
        sendSupFrame(fd, A1, RR(infoFrame));  
        *state = STOP_; 
        return 1;  
    } else {
        sendSupFrame(fd, A1, REJ(infoFrame));  
        *i = 0; 
        return 0;  
    }
}


int llread(unsigned char *packet) {
    unsigned char byte;
    int i = 0;              
    LLState state = START;    
    int bytesRead = 0;         
    unsigned char infoFrame;  
    int stop = 0;           


    while (!stop && state != STOP_) { 
        if (read(fd, &byte, 1) > 0) {
            bytesRead++;
        
            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    break;

                case FLAG_RCV:
                    if (byte == A1) {
                        state = A_RCV;
                    } else if (byte != FLAG) {
                        state = START;
                    }
                    break;

                case A_RCV:
                    if (byte == I(0) || byte == I(1)) {
                        infoFrame = byte; 
                        state = C_RCV;
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                    }else if (byte == SET) {
                        sendSupFrame(fd, A1, UA);
                        state = START;
                        continue;
                    }
                     else if (byte == DISC) {
                        sendSupFrame(fd, A1, DISC);
                        return 0;
                    }
                    else state = START;
                    break;

                case C_RCV:
                    if (byte == (A1 ^ infoFrame)) {
                        state = PROCESSING;
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                        printf("Error on BCC1, going back to START\n");
                        return -1;
                    }
                    break;


                case PROCESSING:
                    if (byte == FLAG) {
                        if (processingData(packet, byte, &i, infoFrame, &state)) {
                            stop = 1; 
                        }
                    } else if (byte == ESC) {
                        state = DETECTED_ESC; 
                    } else {
                        packet[i++] = byte; 
                    }
                    break;

                case DETECTED_ESC:
                    if (byte == ESC || byte == FLAG) {
                        packet[i++] = byte;  
                    } else {
                        packet[i++] = byte ^ 0x20; 
                    }
                    state = PROCESSING;
                    break;

                default:
                    break; 
            }
        } else {
            printf("Read error. Sending REJ.\n");
            sendSupFrame(fd, A1, REJ(infoFrame)); 
            return -1;  
        }
    }

    return i; 
}

int llclose(int showStatistics) {
    LLState state = START;
    initAlarm();  
    alarmCount = 0;
    alarmEnabled = FALSE;

    int discFrameSize = sizeof(A1) + sizeof(DISC) + sizeof(BCC1(A1, DISC));

    while (alarmCount < max_retransmissions) {
        if (!alarmEnabled) {
            sendSupFrame(fd, A1, DISC);  
            alarm(timeout);             
            alarmEnabled = TRUE;

            stats.totalTransmissions++;
            stats.totalBytesSent += discFrameSize; 
        }

        state = SetUaStateMachine(fd, A1, DISC, BCC1(A1, DISC)); 
        if (state == STOP_) {
            stats.successfulTransmissions++;
            alarm(0);
            break;
        }
    }

    if (state != STOP_) {
        printf("Failed after retries.\n");
        stats.failedTransmissions++;
        return -1; 
    }


    int uaFrameSize = sizeof(A1) + sizeof(UA) + sizeof(BCC1(A1, UA));
    sendSupFrame(fd, A1, UA); 
    stats.totalBytesSent += uaFrameSize; 

    printf("Closed with success.\n");

    if (showStatistics) {
        printf("Statistics:\n");
        printf("Total Transmissions: %d\n", stats.totalTransmissions);
        printf("Successful Transmissions: %d\n", stats.successfulTransmissions);
        printf("Failed Transmissions: %d\n", stats.failedTransmissions);
        printf("Total Bytes Sent: %d\n", stats.totalBytesSent);
    }

    int clstat = closeSerialPort();
    return clstat;
}

