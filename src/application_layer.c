// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort,serialPort);
    if (strcmp(role, "tx") == 0) {
        linkLayer.role = LlTx;
    } else {
        linkLayer.role = LlRx;
    }
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;

    int fd = llopen(linkLayer);
    if (fd < 0) {
        perror("Connection error\n");
        exit(-1);
    }
//LlTx abre o ficheiro lê o que tem dentro e separa os dados em packets e dps manda os packets para o Rx  
//LlRx recebe os packets reconstroi o ficheiro e escreve o no disco
    switch(linkLayer.role) {
        case LlTx: {    ////////////////////(slide 27 para construir estes packets)/////////////////////////
            //1. cria um control packet no início para dizer que começou a transmissão (c=1)
            //2. cria data packets enquanto houver data. Acho que o tamanho destes data packets deve ser o MAX_PAYLOAD_SIZE definido no ll.h
            //3. cria um control packet no fim para dizer que acabou a transmissão (c=3) 
            FILE* file = fopen(filename, "rb");
            if (file == NULL) {
                perror("File doesn't exist\n");
                exit(-1);
            }
            fseek(file, 0L, SEEK_END); // https://stackoverflow.com/questions/238603/how-can-i-get-a-files-size-in-c
            long fileSize = ftell(file);//

            unsigned char* ctrlPacket = createCtrlPacket(1); 
            if(llwrite(ctrlPacket, ctrlPacketSize) == -1){
                    perror("Error in ctrlPacket \n");
                    exit(-1);
                }

            //vamos andar de M_P_S em M_P_S entao secalhar fazia dataP = 0 while(dataP <fileSize/M_P_S) dataP ++ ?
            int dataPacketCount = 0;
            int nDataPackets = fileSize/MAX_PAYLOAD_SIZE;

            while(dataPacketCount < nDataPackets){
                unsigned char* dataPacket = createDataPacket();
                if(llwrite(dataPacket, dataPacketSize) == -1){
                    perror("Error in dataPacket \n");
                    exit(-1);
                }
            }

            unsigned char* ctrlPacket = createCtrlPacket(3);
            if(llwrite(ctrlPacket, ctrlPacketSize) == -1){
                    perror("Error in ctrlPacket \n");
                    exit(-1);
                }


            fclose(file); 
            if (llclose(fd)<0) {
                perror("Closing error\n");
                exit(-1);
            }
            break;
        }
        case LlRx: {



            break;
        }
    }


}


unsigned char* createCtrlPacket(int c){
    unsigned char* packet;
    //ver slide 27
    return packet;
}
unsigned char* createDataPacket(){
    unsigned char* packet;
    //ver slide 27
    return packet;
}
