#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <stdbool.h>
#include <signal.h>
#include <winsock.h>
#include "src/nlink_linktrack_anchorframe0.h"
#include "src/nlink_utils.h"

#pragma comment(lib, "ws2_32.lib")

#define BUFFER_SIZE 4096
#define DATA_LENGTH 896
#define START_MARKER 0x55
#define END_MARKER 0xEE
#define BYTES_PER_LINE 22

#define MAX_LINES 1000 

volatile bool stop_signal = false;
SOCKET clientSocket;
struct sockaddr_in serverAddr;

void handle_signal(int signal){
    if (signal == SIGINT || signal == SIGTERM){
        stop_signal = true;
    }
}

void parseLinkTrackData(const uint8_t *data, size_t data_length) {
    if(nlt_anchorframe0_.UnpackData(data, data_length)){
        char buffer[2048] = {0};
        int maxlength = sizeof(buffer);
        int totalLen = 0;

        for (int i = 0; i < nlt_anchorframe0_.result.valid_node_count; i++) {
            nlt_anchorframe0_node_t *node = nlt_anchorframe0_.result.nodes[i];
            int len = snprintf(buffer + totalLen, maxlength - totalLen, "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            node -> id,
            node -> role,
            node -> pos_3d[0],
            node -> pos_3d[1],
            node -> pos_3d[2],
            node -> dis_arr[0],
            node -> dis_arr[1],
            node -> dis_arr[2],
            node -> dis_arr[3],
            node -> dis_arr[4],
            node -> dis_arr[5],
            node -> dis_arr[6],
            node -> dis_arr[7]
            );

            if (len > 0) {
                totalLen += len;
            }
        }

        if (totalLen > 0) {
            int result = sendto(clientSocket, buffer, totalLen, 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
            if (result == SOCKET_ERROR) {
                printf("Failed to send broadcast with error: %d\n", WSAGetLastError());
            }
            else {
                printf("Succesfully received");
                
            }
        }
    } 
    else {
        printf("Parse error\n");
    }
}

bool init_socket_broadcast(int port) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("WSAStartup failed with error: %d\n", WSAGetLastError());
        return false;
    }

    clientSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (clientSocket == INVALID_SOCKET) {
        printf("Socket creation failed with error: %d\n", WSAGetLastError());
        WSACleanup();
        return false;
    }

    // Enable broadcasting
    BOOL broadcast = TRUE;
    if (setsockopt(clientSocket, SOL_SOCKET, SO_BROADCAST, (char*)&broadcast, sizeof(broadcast)) < 0) {
        printf("Failed to set socket as broadcast with error: %d\n", WSAGetLastError());
        closesocket(clientSocket);
        WSACleanup();
        return false;
    }

    // Clear the structure
    memset(&serverAddr, 0, sizeof(serverAddr));
    
    // Set up the broadcast address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);  // Proper network byte order

    printf("Socket initialized for broadcasting on port %d\n", port);
    return true;
}

void cleanup_socket() {
    closesocket(clientSocket);
    WSACleanup();
}

// Process and send data to the socket
void process_frame(const uint8_t *data, size_t length) {
    printf("\nProcessing frame (%zu bytes):\n", length);
    parseLinkTrackData(data, length);
}

HANDLE open_com_port() {
    char port_name[20];
    int com_num;
    
    printf("Enter COM port number (e.g., 11 for COM11): ");
    if (scanf("%d", &com_num) != 1) {
        printf("Invalid input. Program will exit.\n");
        return INVALID_HANDLE_VALUE;
    }
    
    snprintf(port_name, sizeof(port_name), "\\\\.\\COM%d", com_num);
    printf("Attempting to connect to %s...\n", port_name);
    
    HANDLE hComm = CreateFile(port_name, 
                            GENERIC_READ | GENERIC_WRITE, 
                            0, 
                            NULL, 
                            OPEN_EXISTING, 
                            FILE_ATTRIBUTE_NORMAL, 
                            NULL);
    
    if (hComm == INVALID_HANDLE_VALUE) {
        printf("Error: Unable to open %s\n", port_name);
    } else {
        printf("Successfully connected to %s\n", port_name);
    }
    
    return hComm;
}

int main() {
    HANDLE hComm;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
    uint8_t buffer[BUFFER_SIZE] = {0};
    size_t buffer_pos = 0;
    DWORD bytes_read;

    // Set up signal handling
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Initialize socket
    int port = 5000;
    if (!init_socket_broadcast(port)) {
        printf("Failed to initialize socket\n");
        return 1;
    }

    // Get COM port from user and try to open it
    hComm = open_com_port();
    if (hComm == INVALID_HANDLE_VALUE) {
        cleanup_socket();
        return 1;
    }

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(hComm, &dcbSerialParams);
    dcbSerialParams.BaudRate = 921600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.StopBits = ONESTOPBIT;
    SetCommState(hComm, &dcbSerialParams);

    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    SetCommTimeouts(hComm, &timeouts);

    PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
    printf("Reading frames...\n");

    while (!stop_signal) {
        BOOL read_result = ReadFile(hComm, buffer + buffer_pos, 1, &bytes_read, NULL);

        if (!read_result) {
            DWORD error = GetLastError();
            printf("ReadFile error: %lu\n", error);
            continue;
        }

        if (bytes_read > 0) {
            buffer_pos++;

            if (buffer_pos >= DATA_LENGTH) {
                for (size_t i = 0; i <= buffer_pos - DATA_LENGTH; i++) {
                    if (buffer[i] == START_MARKER && buffer[i + DATA_LENGTH - 1] == END_MARKER) {
                        process_frame(buffer + i, DATA_LENGTH);
                        memmove(buffer, buffer + i + DATA_LENGTH, buffer_pos - (i + DATA_LENGTH));
                        buffer_pos -= (i + DATA_LENGTH);
                        break;
                    }
                }
            }

            if (buffer_pos >= BUFFER_SIZE - 1) {
                buffer_pos = 0;
            }
        } else {
            Sleep(1); // Avoid busy looping
        }
    }

    printf("Closing serial port...\n");
    CloseHandle(hComm);

    cleanup_socket();
    printf("Program terminated gracefully.\n");

    return 0;
}