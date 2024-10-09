// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
volatile int STOP = TRUE;

int alarmEnabled = FALSE;
int alarmCount = 0;

/*-----------------------------------------------------------------------*/
////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
//A não perda de informação tem de ser garantida por esta layer
//Os supervision frames sao de comando os unnembered sao de resposta

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Waiting... #%d\n", alarmCount);
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
    // mandar SET e receber UA, supervision and unnembered frames para mandar o SET e o UA
    // TEMOS DE FAZER:
    // 1. Funcões de escrita dos frames    2. Leitura dos frames pela state machine

    unsigned char buf[BUF_SIZE] = {START_S, A1, SET, BCC1(A1,SET), START_S};

    // Set alarm function handler
    (void) signal(SIGALRM, alarmHandler);

    while (alarmCount < 4) {
        if (alarmEnabled == FALSE) {
            int bytes = write(fd, buf, BUF_SIZE);
            printf("%d bytes written\n", bytes);
             for (int i = 0; i < 5; i++) {
                printf("%x ", buf[i]);
            }
            printf("\n");
            alarm(3); 
            alarmEnabled = TRUE;

        }
        int bytes_r = read(fd, buf, BUF_SIZE);
        printf("%d number of bytes read\n", bytes_r);
        for (int i = 0; i < 5; i++) {
            printf("0x%02x ", buf[i]);
        }
        printf("\n");

        if (buf[3] == BCC1(A1, UA)) {
            printf("RECEIVED CORRECTLY\n");
            alarm(0);
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
    //SLIDE 24


    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
