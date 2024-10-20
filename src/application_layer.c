// application.c: // Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <zlib.h> 

unsigned char* createCtrlPacket(int c, int* fileSize, const char *filename);
unsigned char* createDataPacket(int sequence, int dataSize, unsigned char* data);
unsigned char* parseCtrlPacket(unsigned char* packet, int size);
unsigned short calculateChecksum(unsigned char* data, int length);
int verifyChecksum(unsigned char* packet, int packetSize);

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort, serialPort);
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

    switch (linkLayer.role) {
        case LlTx: {
            FILE* file = fopen(filename, "rb");
            if (file == NULL) {
                perror("File doesn't exist\n");
                exit(-1);
            }
            
            fseek(file, 0L, SEEK_END); 
            int fileSize = ftell(file);
            fseek(file, 0L, SEEK_SET); 

            unsigned char* ctrlPacket = createCtrlPacket(1, &fileSize, filename); 
            int ctrlPacketSize = 5 + sizeof(fileSize) + strlen(filename); 
            if (llwrite(ctrlPacket, ctrlPacketSize) == -1) {
                perror("Error in ctrlPacket \n");
                exit(-1);
            }
            printf("APP CTRL PACKET inicial SENT \n");

            int dataPacketCount = 0;
            int nDataPackets = (fileSize + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE; 
            printf("file size %d \n", fileSize);
            printf("DATA PACKETS %d \n", nDataPackets);

            for (int i = 0; i < nDataPackets; ++i) {
                int dataSize = (i == nDataPackets - 1) ? (fileSize % MAX_PAYLOAD_SIZE) : MAX_PAYLOAD_SIZE;
                if (dataSize == 0) dataSize = MAX_PAYLOAD_SIZE;

                unsigned char* data = (unsigned char*) malloc(dataSize);
                fread(data, sizeof(unsigned char), dataSize, file); 

                unsigned char* dataPacket = createDataPacket(dataPacketCount, dataSize, data);
                if (llwrite(dataPacket, 4 + dataSize + sizeof(unsigned short)) == -1) {
                    perror("Error in dataPacket \n");
                    free(data); 
                    exit(-1);
                }
                printf("APP DATA PACKET SENT \n");
                dataPacketCount++;
                free(data); 
            }

            unsigned char* endCtrlPacket = createCtrlPacket(3, &fileSize, filename);
            if (llwrite(endCtrlPacket, ctrlPacketSize) == -1) {
                perror("Error in final ctrlPacket \n");
                exit(-1);
            }
            printf("APP CTRL PACKET final SENT \n");
            fclose(file); 

            if (llclose(fd) < 0) {
                perror("Closing error\n");
                exit(-1);
            }
            break;
        }

        case LlRx: {
            unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE + sizeof(unsigned short));
            if (packet == NULL) {
                perror("Failed to allocate memory for packet");
                exit(-1);
            }

            int packetSize = -1;

            while ((packetSize = llread(packet)) < 0);
            printf("APP READ CTRL PACKET \n");

            if (packetSize < 5) {
                fprintf(stderr, "Error reading control packet\n");
                free(packet);
                exit(EXIT_FAILURE);
            }

            unsigned char* fileName = parseCtrlPacket(packet, packetSize);
            if (fileName == NULL) {
                fprintf(stderr, "Failed to parse control packet\n");
                free(packet);
                exit(EXIT_FAILURE);
            }

            FILE *newFile = fopen("received_file", "wb+"); 
            if (newFile == NULL) {
                perror("Error opening file for writing");
                free(packet); 
                exit(EXIT_FAILURE);
            }

            while (1) {
                while ((packetSize = llread(packet)) < 0);
                if (packetSize == 0) {
                    break; 
                }

                if (verifyChecksum(packet, packetSize)) {
                    if (packet[0] == 2) {
                        printf("APP READ DATA PACKET \n");
                        unsigned char *buffer = (unsigned char *)malloc(packetSize - 4 - sizeof(unsigned short));
                        memcpy(buffer, packet + 4, packetSize - 4 - sizeof(unsigned short));
                        fwrite(buffer, sizeof(unsigned char), packetSize - 4 - sizeof(unsigned short), newFile); 
                        free(buffer); 
                    }
                } else {
                    printf("Checksum error, discarding packet.\n");
                }
            }

            fclose(newFile);
            free(packet);
            break;
        }
    }
}

unsigned char* parseCtrlPacket(unsigned char* packet, int size) {
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

unsigned char* createCtrlPacket(int c, int* fileSize, const char *filename) {
    int fileNameLength = strlen(filename);
    int fileSizeLength = sizeof(*fileSize);

    unsigned char* packet = (unsigned char*)malloc(5 + fileSizeLength + fileNameLength);
    if (!packet) {
        perror("Failed to allocate memory for control packet");
        return NULL;
    }

    packet[0] = c;  
    int i = 1;
    packet[i++] = 0;  
    packet[i++] = fileSizeLength;  
    memcpy(&packet[i], fileSize, fileSizeLength);  
    i += fileSizeLength;

    packet[i++] = 1;  
    packet[i++] = fileNameLength;  
    memcpy(&packet[i], filename, fileNameLength);  

    return packet;
}

unsigned char* createDataPacket(int sequence, int dataSize, unsigned char* data) {
    unsigned char* packet = (unsigned char*)malloc(4 + dataSize + sizeof(unsigned short));
    if (!packet) {
        perror("Failed to allocate memory for data packet");
        return NULL;
    }

    packet[0] = 2; 
    packet[1] = sequence; 
    packet[2] = (dataSize >> 8) & 0xFF; 
    packet[3] = dataSize & 0xFF;

    memcpy(packet + 4, data, dataSize); 

    unsigned short checksum = calculateChecksum(packet, 4 + dataSize);
    memcpy(packet + 4 + dataSize, &checksum, sizeof(unsigned short)); 

    return packet;
}

unsigned short calculateChecksum(unsigned char* data, int length) {
    unsigned short checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

int verifyChecksum(unsigned char* packet, int packetSize) {
    unsigned short receivedChecksum;
    memcpy(&receivedChecksum, packet + packetSize - sizeof(unsigned short), sizeof(unsigned short));
    return calculateChecksum(packet, packetSize - sizeof(unsigned short)) == receivedChecksum;
}

