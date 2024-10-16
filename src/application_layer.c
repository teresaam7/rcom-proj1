// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

unsigned char* createCtrlPacket(int c, int* fileSize, const char *filename);
unsigned char* createDataPacket(int sequence, int dataSize, unsigned char* data);
unsigned char* parseCtrlPacket(unsigned char* packet, int size);

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
            // 1. cria um control packet no início para dizer que começou a transmissão (c=1)
            // 2. cria data packets enquanto houver data. Acho que o tamanho destes data packets deve ser o MAX_PAYLOAD_SIZE definido no ll.h
            // 3. cria um control packet no fim para dizer que acabou a transmissão (c=3) 
            FILE* file = fopen(filename, "rb");
            if (file == NULL) {
                perror("File doesn't exist\n");
                exit(-1);
            }
            fseek(file, 0L, SEEK_END); // https://stackoverflow.com/questions/238603/how-can-i-get-a-files-size-in-c
            int fileSize = ftell(file); // Get file size
            fseek(file, 0L, SEEK_SET); // Reset file position to beginning for reading

            unsigned char* ctrlPacket = createCtrlPacket(1, &fileSize, filename); 
            int ctrlPacketSize = 5 + sizeof(fileSize) + strlen(filename); // c + T1 + T2 + V1 + V2
            if(llwrite(ctrlPacket, ctrlPacketSize) == -1) {
                perror("Error in ctrlPacket \n");
                exit(-1);
            }

            int dataPacketCount = 0;
            int nDataPackets = (fileSize + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;

            for (int i = 0; i < nDataPackets; ++i) {
                int dataSize = (i == nDataPackets - 1) ? (fileSize % MAX_PAYLOAD_SIZE) : MAX_PAYLOAD_SIZE;
                if (dataSize == 0) dataSize = MAX_PAYLOAD_SIZE;

                unsigned char* data = (unsigned char*) malloc(dataSize);
                fread(data, sizeof(unsigned char), dataSize, file); // Read data from file

                unsigned char* dataPacket = createDataPacket(dataPacketCount, dataSize, data);
                if(llwrite(dataPacket, 4 + dataSize) == -1) {
                    perror("Error in dataPacket \n");
                    free(data); // Free data after use
                    exit(-1);
                }
                dataPacketCount++;
                free(data); // Free allocated memory
            }

            unsigned char* endCtrlPacket = createCtrlPacket(3, &fileSize, filename);
            if(llwrite(endCtrlPacket, ctrlPacketSize) == -1) {
                perror("Error in final ctrlPacket \n");
                exit(-1);
            }

            fclose(file); 
            if (llclose(fd) < 0) {
                perror("Closing error\n");
                exit(-1);
            }
            break;
        }
        case LlRx: {
            unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            if (packet == NULL) {
                perror("Failed to allocate memory for packet");
                return exit(-1);
            }

            int packetSize = -1;

            while ((packetSize = llread(fd, packet)) < 0);
            if (packetSize < 0) {
                fprintf(stderr, "Error reading control packet\n");
                free(packet);
                return EXIT_FAILURE;
            }

            unsigned char* fileName = parseCtrlPacket(packet, packetSize);
            if (fileName == NULL) {
                fprintf(stderr, "Failed to parse control packet\n");
                free(packet);
                return EXIT_FAILURE;
            }

            FILE *newFile = fopen((char *)fileName, "wb+");
            if (newFile == NULL) {
                perror("Error opening file for writing");
                free(packet); 
                return EXIT_FAILURE;
            }

            while (1) {
                while ((packetSize = llread(fd, packet)) < 0);
                if (packetSize == 0) {
                    break;  
                }

                if (packet[0] == 2) { 
                    unsigned char *buffer = (unsigned char *)malloc(packetSize - 4);
                    memcpy(buffer, packet + 4, packetSize - 4);
                    fwrite(buffer, sizeof(unsigned char), packetSize - 4, newFile); 
                    free(buffer); 
                }
            }
            
            fclose(newFile);
            free(packet);

            break;
        }
    }
}


unsigned char * parseCtrlPacket(unsigned char* packet, int size){
    unsigned char fileNameNBytes = packet[3 + packet[2] + 1]; 
    unsigned char *fileName = (unsigned char *)malloc(fileNameNBytes + 1); 
    if (fileName == NULL) {
        perror("Failed to allocate memory for file name");
        return NULL; 
    }
    memcpy(fileName, packet + 3 + packet[2] + 2, fileNameNBytes);
    fileName[fileNameNBytes] = '\0'; 

    return fileName;
}

unsigned char* createCtrlPacket(int c, int* fileSize, const char *filename){
    unsigned char* packet = (unsigned char*)malloc(5 + sizeof(*fileSize) + strlen(filename)); // Allocate memory
    if (!packet) {
        perror("Failed to allocate memory for control packet");
        return NULL;
    }
    //ver slide 27
    packet[0] = c;
    int i = 1;
    packet[i++] = 0; //file size
    packet[i++] = sizeof(fileSize);
    packet[i++] = fileSize;
    packet[i++] = 1;
    packet[i++] = sizeof(filename);
    packet[i++] = filename;
    //memcpy(packet + i, filename, strlen(filename)); 

    //T é tipo se é 0, 1, 2...
    //L -> tamanho de cada cena por exemplo se o T é 0, L vai ter o tamanho do fileSize
    //V -> actual fileSize

    return packet;
}
unsigned char* createDataPacket(int sequence, int dataSize, unsigned char* data){
    unsigned char* packet = (unsigned char*)malloc(4 + dataSize); 
    //ver slide 27
    packet[0] = 2;
    packet[1] = sequence;
    packet[2] = dataSize >> 8 & 0xFF;
    packet[3] = dataSize & 0xFF;
    memcpy(packet + 4, data, dataSize);
    return packet;
}



