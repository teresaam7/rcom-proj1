// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
////////////////////////////////////////

///////////////////////////////////////

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


int alarmEnabled = FALSE;
int alarmCount = 0;
int max_retransmissions = 0;
int timeout = 0;

typedef enum
{
   START,
   FLAG_RCV,
   A_RCV,
   C_RCV,
   BCC1_OK,
   BCC2_OK,
   STOP_
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


LLState SetUaStateMachine (int fd, unsigned char expectedAddr, unsigned char expectedCtrl, unsigned char expectedBCC) { //state machine to check and send Supervision and Unnumbered frames, SET and UA
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
                    } else {
                        printf("Byte 0x%02X does not match FLAG. Staying in START.\n", byte);
                    }
                    break;

                case FLAG_RCV:
                    printf("State: FLAG_RCV\n");
                    if (byte == FLAG) {
                        printf("Received FLAG again, staying in FLAG_RCV\n");
                    } else if (byte == expectedAddr) {
                        state = A_RCV;
                        printf("Transition to: A_RCV (A byte received: 0x%02X)\n", byte);
                    } else {
                        state = START;
                        printf("Unexpected byte (0x%02X), returning to START\n", byte);
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
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        return -1;
    }
    int max_retransmissions = connectionParameters.nRetransmissions;
    int timeout = connectionParameters.timeout;

    switch (connectionParameters.role) {
        case LlTx: {
            printf("TRANSMITTER\n");
            (void) signal(SIGALRM, alarmHandler);

            while (alarmCount < max_retransmissions) {
                if (alarmEnabled == FALSE) {
                    sendSupFrame(fd, A1, SET);
                    alarm(timeout); 
                    alarmEnabled = TRUE;
                }

                LLState state = SetUaStateMachine(fd, A2, UA, BCC1(A2, UA));
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
                sendSupFrame(fd, A2, UA);
            }
            break;
        }
    }
    return 1;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{


    // TODO

    /*escrever um pacote e fazer state machine a receber o pacote controlo a dizer que
    (o que foi escrito foi recebido) */
    //slide 23
    /*a state machine do reciever tem de estar preparada para receber um SET e a do Transmiter tem de estar pronto para receber UA. 
    Isto no inicio. se receber um set no meio das information frames é pq algo deu errado*/
    
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO
    //ler o pacote
    //SLIDE 23


    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////



int llclose(int showStatistics)
{
    printf("CLOSING CONNECTION\n");

    (void) signal(SIGALRM, alarmHandler);  
    alarmCount = 0;                        
    alarmEnabled = FALSE;                  

    while (alarmCount <  max_retransmissions ) {
        if (alarmEnabled == FALSE) {
            sendSupervisionFrame(fd, A1, DISC);  
            alarm(timeout); 
            alarmEnabled = TRUE;
        }

        LLState state = SetUaStateMachine(fd, A2, DISC, BCC1(A2, DISC));
        if (state == STOP_) {
            alarm(0);  
            alarmEnabled = FALSE;
            break;
        }

        if (alarmTriggered) {
            printf("Timeout, retrying DISC...\n");
            alarmTriggered = FALSE;  
            alarmCount++;
        }
    }

    if (alarmCount >=  max_retransmissions ) {
        printf("Failed to close connection after retries.\n");
        return -1;  
    }

    sendSupervisionFrame(fd, A1, UA); 
    printf("Connection closed successfully.\n");

    int clstat = closeSerialPort();
    return clstat;
}
