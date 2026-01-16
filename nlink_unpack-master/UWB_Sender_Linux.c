#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include "src/nlink_linktrack_anchorframe0.h"
#include "src/nlink_utils.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h> 
#include <unistd.h> 
#include <termios.h>

#define BUFFER_SIZE 4096
#define DATA_LENGTH 896
#define START_MARKER 0x55
#define END_MARKER 0xEE
#define BYTES_PER_LINE 22

#define MAX_LINES 1000 

volatile bool stop_signal = false;
int clientSocket;
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
            if (result < 0) {
                perror("Faild to send broadcast with error: \n");
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
    clientSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (clientSocket < 0) {
        perror("Socket creation failed with error: \n");
        return false;
    }

    // Enable broadcasting
    int broadcast = 1;
    if (setsockopt(clientSocket, SOL_SOCKET, SO_BROADCAST, (char*)&broadcast, sizeof(broadcast)) < 0) {
        perror("Failed to set socket as broadcast with error: \n");
        close(clientSocket);
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


// Process and send data to the socket
void process_frame(const uint8_t *data, size_t length) {
    printf("\nProcessing frame (%zu bytes):\n", length);
    parseLinkTrackData(data, length);
}

int open_com_port() {
    char port_name[20];
    int com_num;
    
    printf("Enter COM port number (e.g., 11 for COM11): ");
    if (scanf("%d", &com_num) != 1) {
        printf("Invalid input. Program will exit.\n");
        return -1;
    }
    
    snprintf(port_name, sizeof(port_name), "\\\\.\\COM%d", com_num);
    printf("Attempting to connect to %s...\n", port_name);
    
    int hComm = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (hComm < 0) {
        printf("Error: Unable to open %s\n", port_name);
    } else {
        printf("Successfully connected to %s\n", port_name);
    }
    
    return hComm;
}

int main() {
    int hComm;
    uint8_t buffer[BUFFER_SIZE] = {0};
    size_t buffer_pos = 0;
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
    if (hComm < 0) {
        // cleanup_socket();
        return 1;
    }

    struct termios tty;
    if (tcgetattr(hComm, &tty) != 0) {
        printf("Problem occured during initiating the termios\n");
    }

    cfsetospeed(&tty, B921600); // Set output baud rate
    cfsetispeed(&tty, B921600); // Set input baud rate

    tty.c_cflag &= ~PARENB; // Disable parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN]  = 0;  // return as soon as any data is available
    tty.c_cc[VTIME] = 1;
    tcsetattr(hComm,TCSANOW,&tty);
    tcflush(hComm, TCIOFLUSH);
    printf("Reading frames...\n");

    while (!stop_signal) {
        uint8_t byte;
        int read_result = read(hComm, &byte, 1);
        if (read_result > 0) {
            buffer[buffer_pos++] = byte;
            if (buffer_pos >= DATA_LENGTH) {
                for (size_t i = 0; i <= buffer_pos - DATA_LENGTH; i++) {
                    if (buffer[i] == START_MARKER && buffer[i + DATA_LENGTH - 1] == END_MARKER) {
                        process_frame(buffer + i, DATA_LENGTH);
                        memmove(buffer, buffer + i + DATA_LENGTH, buffer_pos - (i + DATA_LENGTH));
                        buffer_pos -= (i + DATA_LENGTH);
                        break;
                    }
                    else {
                    }
                }
            }

            if (buffer_pos >= BUFFER_SIZE - 1) {
                buffer_pos = 0;
            }
        } else {
            sleep(1); // Avoid busy looping
        }
    }

    printf("Closing serial port...\n");
    // CloseHandle(hComm);
    close(hComm);
    close(clientSocket);
    // cleanup_socket();
    printf("Program terminated gracefully.\n");

    return 0;
}