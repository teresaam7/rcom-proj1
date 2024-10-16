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

            unsigned char* ctrlPacket = createCtrlPacket(1, &fileSize, &filename); 
            int ctrlPacketSize = 5 + sizeof(fileSize) + sizeof(filename); // c+ T1 +T2 +V1 + V2
            if(llwrite(ctrlPacket, ctrlPacketSize) == -1){
                    perror("Error in ctrlPacket \n");
                    exit(-1);
                }

            int dataPacketCount = 0;
            int nDataPackets = (fileSize + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;

            for (int i = 0; i < nDataPackets; ++i) {
            int dataSize;
            if (i == nDataPackets - 1) {
                dataSize = fileSize % MAX_PAYLOAD_SIZE;
                if (dataSize == 0) dataSize = MAX_PAYLOAD_SIZE;
                else dataSize = MAX_PAYLOAD_SIZE;

                unsigned char* data = (unsigned char*) malloc(dataSize);
                int dataPacketSize = 4 + dataSize;
                unsigned char* dataPacket = createDataPacket(dataPacketCount, dataSize, data);
                if(llwrite(dataPacket, dataPacketSize) == -1){
                    perror("Error in dataPacket \n");
                    exit(-1);
                }
            dataPacketCount++;
            }

            unsigned char* ctrlPacket = createCtrlPacket(3, &fileSize, &filename);
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
            //read ctrl
            unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            while(llread(packet))
            //read data e escrever num file
            //read ctrl final

            
            break;
            }
        }

    }
}

unsigned char* createCtrlPacket(int c, int* fileSize, const char *filename){
    unsigned char* packet;
    //ver slide 27
    packet[0] = c;
    int i = 1;
    packet[i++] = 0; //file size
    packet[i++] = sizeof(fileSize);
    packet[i++] = fileSize;
    packet[i++] = 1;
    packet[i++] = sizeof(filename);
    packet[i++] = filename;

    //T é tipo se é 0, 1, 2...
    //L -> tamanho de cada cena por exemplo se o T é 0, L vai ter o tamanho do fileSize
    //V -> actual fileSize

    return packet;
}
unsigned char* createDataPacket(int sequence, int dataSize, unsigned char* data){
    unsigned char* packet;
    //ver slide 27
    packet[0] = 2;
    packet[1] = sequence;
    packet[2] = dataSize >> 8 & 0xFF;
    packet[3] = dataSize & 0xFF;
    memcpy(packet + 4, data, dataSize);
    return packet;
}
