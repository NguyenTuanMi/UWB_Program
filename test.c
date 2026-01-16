#include <stdio.h>

int main() {
    char buffer[50];
    char* s = "geeksforgeeks";
    size_t size = sizeof(buffer);
    int len = 0;
    for(int i=0; i<=size; i++) {
        int j = snprintf(buffer + len, size - len, "%s", s);
        printf("This is the buffer %s \n", buffer);
        printf("This is the value of j: %d\n", j);
        printf("The value of i: %d\n", i);
        printf("The value of max len: %d\n", size-len);
        if (j>0) {
            len += j;
        }
        // If the pointer surpasses the size of the buffer, it wouldn't wrap around and it would move to an undefined memory, the behaviour would be unpredictable. 
    }
    return 0;
}