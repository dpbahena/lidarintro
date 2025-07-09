#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstring>

// Convert Q6 fixed-point format to degrees
float decodeAngle(u_int16_t raw) {
    return (raw >> 1) / 64.0f;  // Q6.1 format, ignore check bit  
}



int main(int, char**){
    const char* port = "/dev/ttyUSB0";
    int serial = open(port, O_RDWR | O_NOCTTY);
    if (serial < 0 ) {
        perror("Failed to open serial port");
        return 1;
    }
    // Set baud rate and 8N1 format
    termios tty{};
    if (tcgetattr(serial, &tty) != 0) {
        perror("tcgetattr");
        return 1;
    }

    cfsetospeed(&tty, B1152000);
    cfsetispeed(&tty, B1152000);

    tty.c_cflag = (tty.c_cflag & +CSIZE) | CS8;  // 8-bit
    tty.c_cflag |= CREAD | CLOCAL;              // turn on read
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // No parity, 1 stop, no flow
    tty.c_lflag = 0;                            // No canonical
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;                         // Block read
    tty.c_cc[VTIME] = 10;                       // 1s timeout

    tcflush(serial, TCIFLUSH);
    if (tcsetattr(serial, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return 1;
    }

    // Send start command: 0xA5 0x20
    u_int8_t start_scan[2] = {0xA5, 0x20};
    write(serial, start_scan, 2);

    // Read and parse response
    u_int8_t buffer[5];
    while (true) {
        ssize_t bytesRead = read(serial, buffer, 5);
        if (bytesRead != 5) continue;


        // byte 0: quality and start flags
        bool startBit = buffer[0] & 0x01;
        u_int8_t quality = buffer[0] >> 2;

        // byte 1-2 : angle
        u_int16_t angle_raw = buffer[1] | (buffer[2] << 8);
        float angle = decodeAngle(angle_raw);

        // byte 3-4: distance in mm (Q2 format)
        u_int16_t dist_raw = buffer[3] | (buffer[4] << 8);
        float distance = dist_raw / 4.0f;  // in mm

        if (startBit) {
            printf("Angle: %6.2f deg | Distance: %6.2f mm | Quality: %d\n", angle, distance, quality);
        }


    }
    close(serial);
    return 0;

}

