// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
/*-----------------------------------------------------------------------*/
#define START_S 0x7E // start of a supervision frame
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
#define         0x00
/*-----------------------------------------------------------------------*/
////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
//A não perda de informação tem de ser garantida por esta layer
//Os supervision frames sao de comando os unnembered sao de resposta
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {

        return -1;
    }

    // TODO
    //slide 10, 22
    //mandar SET e receber UA, supervision and unnembered frames para mandar o SET e o UA

    unsigned char buf[BUF_SIZE] = {FLAG, ADDR, CTRL_SET, BCC1(ADDR,CTRL_SET), FLAG};


    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

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

        if (buf[3] == BCC1(ADDR, CTRL_UA)) {
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
