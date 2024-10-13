// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
////////////////////////////////////////

///////////////////////////////////////

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
volatile int STOP = TRUE;

int alarmEnabled = FALSE;
int alarmCount = 0;

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


void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Waiting... #%d\n", alarmCount);
}

int sendSupFrame(int fd, unsigned char addr, unsigned char ctrl){
    unsigned char frame[BUF_SIZE] = {FLAG, addr, ctrl, addr ^ ctrl, FLAG};
    int bytes = write(fd, frame, BUF_SIZE);
    printf("SSF %d bytes written\n", bytes);
    for (int i = 0; i < 5; i++) {
        printf("%x ", frame[i]);
    }
    printf("\n");
    return bytes;
}

int llopen(LinkLayer connectionParameters)
{
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0) {

        return -1;
    }

    // TODO
    // slide 10, 22
    // mandar SET e receber UA, supervision and unnumbered frames para mandar o SET e o UA
    // TEMOS DE FAZER:
    // 1. Funcões de escrita dos frames    2. Leitura dos frames pela state machine

    
       LLState state = START;
       int max_retransmissions = connectionParameters.nRetransmissions;
       int timeout = connectionParameters.timeout;
       switch(connectionParameters.role) {
        case LlRx: {
            printf("RECIEVER");
            unsigned char buf[BUF_SIZE] = {0};
            while (STOP == FALSE) {
                int bytes = read(fd, buf, BUF_SIZE);
                printf("R recieved = 0x%02X\n", buf[2]);

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
                                state = FLAG_RCV;  
                            } else if (byte == A1) {
                                state = A_RCV;
                                printf("Transition to: A_RCV (A byte received)\n");
                            } else {
                                state = START;  
                                printf("Unexpected byte, returning to START\n");
                            }
                            break;

                        case A_RCV:
                            printf("State: A_RCV\n");
                            if (byte == FLAG) {
                                state = FLAG_RCV;  
                                printf("Received FLAG, transition to: FLAG_RCV\n");
                            } else if (byte == SET) {
                                state = C_RCV;
                                printf("Transition to: C_RCV (C byte received)\n");
                            } else {
                                state = START;  
                                printf("Unexpected byte, returning to START\n");
                            }
                            break;

                        case C_RCV:
                            printf("State: C_RCV\n");
                            if (byte == FLAG) {
                                state = FLAG_RCV;  
                                printf("Received FLAG, transition to: FLAG_RCV\n");
                            } else if (byte == BCC1(A1, SET)) {
                                state = BCC1_OK;
                                printf("Transition to: BCC1_OK (BCC1 check passed)\n");
                            } else {
                                state = START; 
                                printf("BCC1 mismatch, returning to START\n");
                            }
                            break;

                        case BCC1_OK:
                            printf("State: BCC1_OK\n");
                            if (byte == FLAG) {
                                state = STOP_;
                                sendSupFrame(fd, A2, UA);
                                STOP = TRUE;
                                printf("Transition to: STOP (valid frame received)\n");
                            } else {
                                state = START;  
                                printf("Expected FLAG but received something else, returning to START\n");
                            }
                            break;

                        default:
                            printf("Unknown state, resetting to START\n");
                            state = START; 
                            break;
                    }
                }
            }
            break;
        }
        case LlTx: {
            printf("TRANSMITTER");
            unsigned char buf[BUF_SIZE] = {FLAG, A1, SET, BCC1(A1,SET), FLAG};

            (void) signal(SIGALRM, alarmHandler);

            while (alarmCount < max_retransmissions) {
                if (alarmEnabled == FALSE) {
                    int bytes = sendSupFrame(fd, A1, SET);
                    printf("T %d bytes written\n", bytes);
                    for (int i = 0; i < 5; i++) {
                        printf("%x ", buf[i]);
                    }
                    printf("\n");
                    alarm(timeout); 
                    alarmEnabled = TRUE;

                }
                int bytes_r = read(fd, buf, BUF_SIZE);
                printf("T %d number of bytes read\n", bytes_r);
                for (int i = 0; i < 5; i++) {
                    printf("0x%02x ", buf[i]);
                }
                printf("\n");

                if (buf[3] == BCC1(A2, UA)) { //Fazer aqui verificação da receção de frames UA
                    printf("RECEIVED CORRECTLY\n");
                    alarm(0);
                    break;
                }
            }
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
    // TODO
    //slide 24
    int clstat = closeSerialPort();
    return clstat;
}
