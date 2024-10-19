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
int sendSupFrame(int fd, unsigned char addr, unsigned char ctrl);
LLState SetUaStateMachine(int fd, unsigned char expectedAddr, unsigned char expectedCtrl, unsigned char expectedBCC);
unsigned char calculateBCC2(const unsigned char *buf, int bufSize);
void byteStuffingTechnique(unsigned char **frame, int *frameSize, unsigned char byte, int *j);
unsigned char infoFrameStateMachine(int fd);
int processingData(int fd, unsigned char *packet, unsigned char byte, int *i, unsigned char *bcc2);

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
    //unsigned char byte;
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
            printf("Byte read: 0x%02X (State: %d)\n", byte, state);
        switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;  
                    printf("State transitioned to FLAG_RCV.\n");
                }
                break;
            case FLAG_RCV:
                if (byte == A2) {
                    state = A_RCV; 
                    printf("State transitioned to A_RCV.\n");
                } else if (byte != FLAG) {
                    state = START;
                    printf("Invalid byte received. Transitioned back to START.\n");
                }
                break;
            case A_RCV:
                if (byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1 || byte == DISC) {
                    infoFrame = byte;  
                    state = C_RCV;  
                    printf("State transitioned to C_RCV with infoFrame: 0x%02X.\n", infoFrame);
                } else if (byte == FLAG) {
                    state = FLAG_RCV; 
                    printf("State transitioned to FLAG_RCV.\n");
                } else {
                    state = START;
                    printf("Invalid byte received. Transitioned back to START1.\n");
                }
                break;
            case C_RCV:
                if (byte == BCC1(A2, infoFrame)) {
                    state = BCC1_OK;
                    printf("State transitioned to BCC1_OK.\n");
                } else if (byte == FLAG) {
                    state = FLAG_RCV; 
                    printf("State transitioned to FLAG_RCV.\n");
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
                    printf("Invalid byte received. Transitioned back to START.\n");
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
    unsigned char BCC2 = calculateBCC2(buf, bufSize);

    int j = 4; // Start after A, C and BCC1
    for (int i = 0; i < bufSize; i++) {
        byteStuffingTechnique(&frame, &totalSize, buf[i], &j);
    }

    byteStuffingTechnique(&frame, &totalSize, BCC2, &j);
    

    //frame[j++] = BCC2;
    frame[j++] = FLAG;

    int transmission = 0;
    int check_rej = 0, check_rr = 0;

    printf("Starting transmission with a maximum of %d attempts...\n", max_retransmissions);

    for(int i = 0; i<j; i++){
        printf(" 0x%02X", frame[i]);
        printf("\n");
    }
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

int processingData(int fd, unsigned char *packet, unsigned char byte, int *i, unsigned char *bcc2) {
    if (byte == ESC) {
        printf("ESC detected. Transitioning to DETECTED_ESC.\n");
        return DETECTED_ESC; 
    } else if (byte == FLAG) {
        *bcc2 = packet[*i - 1]; 
        (*i)--; 
        packet[*i] = '\0'; 

        unsigned char bcc2_res = calculateBCC2(packet, *i);
        printf("Received FLAG. Comparing BCC2. Calculated BCC2: 0x%02X, Received BCC2: 0x%02X\n", bcc2_res, *bcc2);

        if (*bcc2 == bcc2_res) {
            printf("BCC2 match. Transitioning to STOP_.\n");
            sendSupFrame(fd, A2, RR(receive));
            return STOP_; 
        } else {
            printf("Error: retransmission required.\n");
            sendSupFrame(fd, A2, REJ(receive));
            return -1; 
        }
    } else {
        packet[*i] = byte; 
        (*i)++; 
        printf("Processing byte: 0x%02X. Current index: %d\n", byte, *i);
        return PROCESSING; 
    }
}

int llread(unsigned char *packet) {
    unsigned char byte, bcc2;
    int i = 0;  // Índice para o buffer de dados do pacote
    LLState state = START;
    int bytesRead = 0;
    unsigned char infoFrame;
    int stop = 0;  // Variável para saber se a leitura terminou

    printf("Iniciando llread...\n");

    while (!stop) {  // Vamos garantir que lemos até o final
        if (read(fd, &byte, 1) > 0) {
            printf("Byte recebido: 0x%02X\n", byte);  // Depuração para cada byte recebido
            bytesRead++;
            
            switch (state) {
                case START:
                    printf("Estado: START\n");
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        printf("FLAG detectada, indo para FLAG_RCV\n");
                    }
                    break;

                case FLAG_RCV:
                    printf("Estado: FLAG_RCV\n");
                    if (byte == A1) {
                        state = A_RCV;
                        printf("A1 detectado, indo para A_RCV\n");
                    } else if (byte != FLAG) {
                        state = START;
                        printf("Byte diferente de FLAG e A1, voltando para START\n");
                    }
                    break;

                case A_RCV:
                    printf("Estado: A_RCV\n");
                    if (byte == I(0) || byte == I(1)) {
                        infoFrame = byte;  // Captura o número de sequência do quadro
                        state = C_RCV;
                        printf("Frame de informação detectado: 0x%02X, indo para C_RCV\n", byte);
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                        printf("FLAG detectada, indo para FLAG_RCV\n");
                    }
                    break;

                case C_RCV:
                    printf("Estado: C_RCV\n");
                    if (byte == (A1 ^ infoFrame)) {
                        state = PROCESSING;
                        printf("BCC1 verificado com sucesso, indo para PROCESSING\n");
                    } else {
                        state = START;
                        printf("Erro no BCC1, voltando para START\n");
                    }
                    break;

                case PROCESSING:
                    printf("Estado: PROCESSING\n");
                    if (byte == FLAG) {
                        printf("FLAG detectada, fim do quadro de dados\n");
                        bcc2 = packet[i - 1];  // BCC2 é o último byte antes do FLAG
                        i--;  // Excluir o BCC2 do comprimento do pacote
                        printf("Verificando BCC2...\n");

                        unsigned char calculatedBCC2 = calculateBCC2(packet, i);
                        if (bcc2 == calculatedBCC2) {
                            printf("BCC2 verificado com sucesso. Enviando RR.\n");
                            sendSupFrame(fd, A2, RR(infoFrame));  // Enviar ACK
                            state = STOP_;  // Marcar o estado de fim
                            stop = 1;  // Indicar que recebemos o pacote completo
                        } else {
                            printf("Erro no BCC2, enviando REJ.\n");
                            sendSupFrame(fd, A2, REJ(infoFrame));  // Rejeitar pacote
                            i = 0;  // Resetar o índice do buffer de pacote
                        }
                    } else if (byte == ESC) {
                        printf("ESC detectado, indo para DETECTED_ESC\n");
                        state = DETECTED_ESC;
                    } else {
                        printf("Byte de dados armazenado: 0x%02X\n", byte);
                        packet[i++] = byte;  // Armazenar byte de dados
                    }
                    break;

                case DETECTED_ESC:
                    printf("Estado: DETECTED_ESC\n");
                    if (byte == ESC || byte == FLAG) {
                        printf("Byte escapado detectado: 0x%02X\n", byte);
                        packet[i++] = byte;
                    } else {
                        printf("Byte escapado decodificado: 0x%02X\n", byte ^ 0x20);
                        packet[i++] = byte ^ 0x20;  // Desfazer byte stuffing
                    }
                    state = PROCESSING;  // Continuar processando
                    break;

                default:
                    break;
            }
        } else {
            printf("Erro na leitura. Enviando REJ.\n");
            sendSupFrame(fd, A2, REJ(infoFrame));  // Solicitar retransmissão
            return -1;  // Erro de leitura, abortar
        }
    }

    printf("Pacote recebido com sucesso. Tamanho: %d bytes\n", i);
    return i;  // Retornar o número de bytes lidos (tamanho do pacote)
}



////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose(int showStatistics)
{
    LLState state = START;
    (void) signal(SIGALRM, alarmHandler);  
    alarmCount = 0;                       
    alarmEnabled = FALSE;              

    while (alarmCount < max_retransmissions) {
        if (!alarmEnabled) {
            sendSupFrame(fd, A1, DISC);  
            alarm(timeout);             
            alarmEnabled = TRUE;
        }

        state = SetUaStateMachine(fd, A2, DISC, BCC1(A2, DISC)); 
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
