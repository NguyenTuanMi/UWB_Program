#include <stdio.h>
#include <stdint.h>
#include <Windows.h>
#include <src/nlink_linktrack_anchorframe0.h>

DWORD win32_error_code;
CHAR error_message[256];

void parseLinkTrackData(const uint8_t data, size_t dataLength) {
    if(nlt_anchorframe0_.UnpackData(data, dataLength)) {
        char stored[2048] = {0}; // Size of the buffer must be big enough 
        size_t size = sizeof(stored);
        int len = 0; 

        for (int i=0; i<nlt_anchorframe0_.result.valid_node_count; i++) {
            nlt_anchorframe0_node_t *nodes = nlt_anchorframe0_.result.nodes[i];
            int j = sprintf(stored + len, size - len, 
                "%d", "%s", 
                "%.2f", "%.2f", "%.2f", 
                "%d", "%d", "%d", "%d", "%d", "%d", "%d", "%d",
                nodes -> id, 
                nodes -> role, 
                nodes -> pos_3d[0], 
                nodes -> pos_3d[1], 
                nodes -> pos_3d[2],
                nodes -> dis_arr[0],
                nodes -> dis_arr[1],
                nodes -> dis_arr[2],
                nodes -> dis_arr[3],
                nodes -> dis_arr[4],
                nodes -> dis_arr[5],
                nodes -> dis_arr[6],
                nodes -> dis_arr[7]
            );

            if (j > 0) {
                len += j;
            }
        }
    }
}

int main() {
    HANDLE hComm = CreateFile(     // HANDLE object represents several OS-managed resources, such as events, threads, window, files. This thing creates and opens a COMM port's file?)
        "\\\\.\\COM3",
        GENERIC_READ | GENERIC_WRITE, 
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (hComm == INVALID_HANDLE_VALUE) { 
        printf("Error in opening serial port\n");
        win32_error_code = GetLastError(); 
        
        FormatMessage(
            FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
            NULL, 
            win32_error_code,
            0,
            error_message,
            sizeof(error_message),
            NULL
        );
        printf("\nError: %s", error_message);
    }
    else {
        printf("Openning successfully\n");
    }

    // Formating the control settings of serial communication port.
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hComm, &dcbSerialParams)) {
        printf("\nError in receiving the communication state");
    }
    
    dcbSerialParams.BaudRate = 921600; // Example baud rate
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hComm, &dcbSerialParams)) {
        printf("\nError in setting the communication state");
    }

    BOOL success;
    DWORD bytesRead;
    char receive_buffer[2048] = {0};
    success = ReadFile(
        hComm, 
        receive_buffer,
        sizeof(receive_buffer),
        &bytesRead,
        NULL
    );

    COMMTIMEOUTS timeouts = {0}; //Zero out the COMMTIMEOUTS structure
    timeouts.ReadIntervalTimeout        = 20; // Max time between bytes (ms)
    timeouts.ReadTotalTimeoutMultiplier = 1;  // Per-byte timeout
    timeouts.ReadTotalTimeoutConstant   = 50; // Constant timeout (ms)
    timeouts.WriteTotalTimeoutMultiplier = 1; // Per-byte timeout
    timeouts.WriteTotalTimeoutConstant   = 50;// Constant timeout (ms)
    SetCommTimeouts(hComm, &timeouts); //finally set the timeouts
    
    CloseHandle(hComm);
    return 0;
}