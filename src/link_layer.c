// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
////////////////////////////////////////

///////////////////////////////////////

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

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


#define T_SECONDS 3
#define BUF_SIZE 5 // POSIX compliant source

/*-----------------------------------------------------------------------*/

#define FLAG    0x7E // start of a supervision frame
#define A1      0x03 // addr field in command frames(S) sent by the Transmitter or replies(U) sent by the reciever
#define A2      0x01 // addr field in command frames(S) sent by the Reciever or replies(U) sent by the Transmiter
#define ESC     0x7D // escape octet followed by the result of the exclusive

//------Supervision and Unnumbered frames -----------//

#define SET     0X03
#define UA      0x07
#define RR0     0xAA // reciever is ready to recieve an information frame nº 0
#define RR1     0xAB // reciever is ready to recieve an information frame nº 1
#define REJ0    0x54 // reciever is rejects information from frame nº 0
#define REJ1    0x55 // reciever is rejects information from frame nº 1
#define DISC    0x0B // frame to indicate termination of a connection
#define BCC1(a,c) ((a)^(c)) // field to detect the occurence of errors in the header

//------Information frames -----------//

#define INFO0    0x00 // Information frame number 0
#define INFO1    0x80 // Information frame number 1

// Nr- Número de sequências de receção
// Ns- Número de sequências de transmissão

#define RR(Nr) ((Nr == 0) ? RR0 : RR1)
#define REJ(Nr) ((Nr == 0) ? REJ0 : REJ1)
#define I(Ns)  ((Ns == 0) ? INFO0 : INFO1)

/*-----------------------------------------------------------------------*/
// Function Prototypes
void alarmHandler(int signal);
void initAlarm();
int sendSupFrame(int fd, unsigned char addr, unsigned char ctrl);
LLState SetUaStateMachine(int fd, unsigned char expectedAddr, unsigned char expectedCtrl, unsigned char expectedBCC);
unsigned char calculateBCC2(const unsigned char *buf, int bufSize);
void byteStuffingTechnique(unsigned char **frame, int *frameSize, unsigned char byte, int *j);
unsigned char infoFrameStateMachine(int fd);

/*-----------------------------------------------------------------------*/
////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
//A não perda de informação tem de ser garantida por esta layer
//Os supervision frames sao de comando os unnembered sao de resposta
//I,S ou U frames com header errado são ignorados
//Só o transmitor envia I frames

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
    printf("SSF %d bytes written\n", bytes);
    for (int i = 0; i < 5; i++) {
        printf("%x ", frame[i]);
    }
    printf("\n");
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
        printf("Received Bytes: ");
        for (int i = 0; i < bytes; i++) {
            printf("0x%02X ", buf[i]);
        }
        printf("\n");

        for (int i = 0; i < bytes; i++) {
            unsigned char byte = buf[i];
            switch (state) {
                case START:
                    printf("State: START\n");
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        printf("Transition to: FLAG_RCV\n");
                    }
                    break;

                case FLAG_RCV:
                    printf("State: FLAG_RCV\n");
                    if (byte == FLAG) {
                        printf("Received FLAG again, staying in FLAG_RCV\n");
                        state = START;
                    } else if (byte == expectedAddr) {
                        state = A_RCV;
                        printf("Transition to: A_RCV (A byte received: 0x%02X)\n", byte);
                    }
                    break;

                case A_RCV:
                    printf("State: A_RCV\n");
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        printf("Received FLAG, transition to: FLAG_RCV\n");
                    } else if (byte == expectedCtrl) {
                        state = C_RCV;
                        printf("Transition to: C_RCV (C byte received: 0x%02X)\n", byte);
                    } else {
                        state = START;
                        printf("Unexpected byte (0x%02X), returning to START\n", byte);
                    }
                    break;

                case C_RCV:
                    printf("State: C_RCV\n");
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        printf("Received FLAG, transition to: FLAG_RCV\n");
                    } else if (byte == expectedBCC) {
                        state = BCC1_OK;
                        printf("Transition to: BCC1_OK (BCC1 check passed: 0x%02X)\n", byte);
                    } else {
                        state = START;
                        printf("BCC1 mismatch (received 0x%02X), returning to START\n", byte);
                    }
                    break;

                case BCC1_OK:
                    printf("State: BCC1_OK\n");
                    if (byte == FLAG) {
                        state = STOP_;
                        printf("Transition to: STOP_  |!RECEIVED CORRECTLY!| \n");
                    } else {
                        state = START;
                        printf("Expected FLAG but received 0x%02X, returning to START\n", byte);
                    }
                    break;

                default:
                    printf("Unknown state encountered. Resetting to START.\n");
                    state = START;
                    break;
            }
        }
    }
    return state;
}

int llopen(LinkLayer connectionParameters){
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        return -1;
    }
    max_retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    switch (connectionParameters.role) {
        case LlTx: {
            printf("TRANSMITTER\n");
            

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
                    break;
                }
            }
            break;
        }
        case LlRx: {
            printf("RECEIVER\n");
            LLState state = SetUaStateMachine(fd, A1, SET, BCC1(A1, SET));
            if (state == STOP_) {
                sendSupFrame(fd, A1, UA);
            }
            break;
        }
    }
    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

 /*escrever um pacote e fazer state machine a receber o pacote controlo a dizer que
    (o que foi escrito foi recebido) */
    //slide 23
    /*a state machine do reciever tem de estar preparada para receber um SET e a do Transmiter tem de estar pronto para receber UA. 
    Isto no inicio. se receber um set no meio das information frames é pq algo deu errado*/

unsigned char calculateBCC2(const unsigned char *buf, int bufSize) {
    unsigned char BCC2 = buf[0];
    for (int i = 1; i < bufSize; i++) {
        BCC2 ^= buf[i];
    }
    return BCC2;
}

void byteStuffingTechnique(unsigned char **frame, int *frameSize, unsigned char byte, int *j) {
    if (byte == FLAG || byte == ESC) {
        *frame = (unsigned char *)realloc(*frame, ++(*frameSize)); 
        (*frame)[(*j)++] = ESC;
    }
    (*frame)[(*j)++] = byte;
}


unsigned char infoFrameStateMachine(int fd) {
    unsigned char infoFrame = 0;
    LLState state = START;

    printf("Starting infoFrameStateMachine...\n");
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
                        printf("State transitioned to BCC1_OK.\n");
                    } else if (byte == FLAG) {
                        state = FLAG_RCV; 
                    } else {
                        state = START; 
                        printf("Invalid BCC1 received. Transitioned back to START.\n");
                    }
                    break;
                case BCC1_OK:
                    if (byte == FLAG) {
                        state = STOP_;
                        printf("State transitioned to STOP_. Frame complete.\n");
                    } else {
                        state = START; 
                    }
                    break;
                default:
                    state = START;
                    printf("Unknown state. Transitioned back to START.\n");
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


    int j = 4; // Start after A, C and BCC1
    for (int i = 0; i < bufSize; i++) {
        byteStuffingTechnique(&frame, &totalSize, buf[i], &j);
    }

    unsigned char BCC2 = calculateBCC2(buf, bufSize);
    byteStuffingTechnique(&frame, &totalSize, BCC2, &j);
    
    frame[j++] = FLAG;

    int transmission = 0;
    int check_rej = 0, check_rr = 0;

    printf("Starting transmission with a maximum of %d attempts...\n", max_retransmissions);

   
    while (transmission < max_retransmissions) { 
        alarmEnabled = FALSE; 
        alarm(timeout);       
        check_rej = 0;
        check_rr = 0;

        while (!alarmEnabled && !check_rej && !check_rr) {
            printf("Writing frame to fd...\n");
            write(fd, frame, j);

            unsigned char res = infoFrameStateMachine(fd); 

            if (!res) {
                continue;
            } else if (res == REJ(0) || res == REJ(1)) { 
                check_rej = 1;
                printf("Received REJ response.\n");
            } else if (res == RR(0) || res == RR(1)) { 
                check_rr = 1;
                send = (send + 1) % 2; 
                printf("Received RR response. Transitioning send to %d.\n", send);
            } else {
                continue;
            }
        }

        if (check_rr) {
            printf("Transmission successful.\n");
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

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////

int processingData(unsigned char *packet, unsigned char byte, int *i, unsigned char infoFrame, LLState *state) {
    unsigned char bcc2 = packet[*i - 1];
    (*i)--; 
    printf("Checking BCC2...\n");

    unsigned char calculatedBCC2 = calculateBCC2(packet, *i);
    if (bcc2 == calculatedBCC2) {
        printf("BCC2 verified successfully. Sending RR.\n");
        sendSupFrame(fd, A1, RR(infoFrame));  
        *state = STOP_; 
        return 1;  
    } else {
        printf("Error in BCC2, sending REJ.\n");
        sendSupFrame(fd, A1, REJ(infoFrame));  
        *i = 0; 
        return 0;  
    }
}


int llread(unsigned char *packet) {
    unsigned char byte;
    int i = 0;                 // Index for the packet buffer
    LLState state = START;    
    int bytesRead = 0;         
    unsigned char infoFrame;  
    int stop = 0;              // Flag to determine when to stop reading

    printf("Starting llread...\n");

    while (!stop && state != STOP_) { 
        if (read(fd, &byte, 1) > 0) {
            printf("Byte received: 0x%02X\n", byte);
            bytesRead++;
        
            switch (state) {
                case START:
                    printf("State: START\n");
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        printf("FLAG detected, transitioning to FLAG_RCV\n");
                    }
                    break;

                case FLAG_RCV:
                    printf("State: FLAG_RCV\n");
                    if (byte == A1) {
                        state = A_RCV;
                        printf("A1 detected, transitioning to A_RCV\n");
                    } else if (byte != FLAG) {
                        state = START;
                        printf("Byte different from FLAG and A1, returning to START\n");
                    }
                    break;

                case A_RCV:
                    printf("State: A_RCV\n");
                    if (byte == I(0) || byte == I(1)) {
                        infoFrame = byte; 
                        state = C_RCV;
                        printf("Info frame detected: 0x%02X, transitioning to C_RCV\n", byte);
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                        printf("FLAG detected, transitioning to FLAG_RCV\n");
                    }else if (byte == SET) {
                        sendSupFrame(fd, A1, SET);
                        return 0;
                    }
                     else if (byte == DISC) {
                        sendSupFrame(fd, A1, DISC);
                        return 0;
                    }
                    else state = START;
                    break;

                case C_RCV:
                    printf("State: C_RCV\n");
                    if (byte == (A1 ^ infoFrame)) {
                        state = PROCESSING;
                        printf("BCC1 verified successfully, transitioning to PROCESSING\n");
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                        printf("Error in BCC1, returning to START\n");
                    }
                    break;

                case PROCESSING:
                    printf("State: PROCESSING\n");
                    if (byte == FLAG) {
                        printf("FLAG detected, end of data frame\n");
                        if (processingData(packet, byte, &i, infoFrame, &state)) {
                            stop = 1;  // Stop reading if the packet was processed successfully
                        }
                    } else if (byte == ESC) {
                        state = DETECTED_ESC; 
                    } else {
                        packet[i++] = byte; 
                    }
                    break;

                case DETECTED_ESC:
                    printf("State: DETECTED_ESC\n");
                    if (byte == ESC || byte == FLAG) {
                        printf("Escaped byte detected: 0x%02X\n", byte);
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

    printf("Packet received successfully. Size: %d bytes\n", i);
    return i; 
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose(int showStatistics)
{
    LLState state = START;
    initAlarm();  
    alarmCount = 0;                       
    alarmEnabled = FALSE;              

    while (alarmCount < max_retransmissions) {

        if (!alarmEnabled) {
            sendSupFrame(fd, A1, DISC);  
            alarm(timeout);             
            alarmEnabled = FALSE;
        }

        state = SetUaStateMachine(fd, A1, DISC, BCC1(A1, DISC)); 
        if (state == STOP_) {
            alarm(0);  
            break;
        }
    }

    if (state != STOP_) {
        printf("FAILED AFTER RETRIES.\n");
        return -1; 
    }

    sendSupFrame(fd, A1, UA); 
    printf("CLOSED WITH SUCCESS.\n");

    int clstat = closeSerialPort();
    return clstat;
}
