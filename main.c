#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <raylib.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#define WIDTH 800
#define HEIGHT 600

int serial_port_open(const char *devpath) {
    int serial_port;
    struct termios tty;

    serial_port = open(devpath, O_RDWR);
    if(serial_port < 0) {
        perror("open");
        return -1;
    }
    if(tcgetattr(serial_port, &tty) != 0) {
        perror("tcgetattr");
        close(serial_port);
        return -1;
    }
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    /*tty.c_cflag &= ~CRTSCTS;*/
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(serial_port);
        return -1;
    }
    return serial_port;
}

int serial_port_close(int serial_port) {
    close(serial_port);
    return -1;
}

int main(void)
{
    int comboBoxActive = 1;

    int serial_port = - 1;
    static int graph[WIDTH] = {0}, gptr = 0;

    InitWindow(WIDTH, HEIGHT, "microplotter");

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();

            ClearBackground(RAYWHITE);
            if(serial_port < 0) {
                if (GuiButton((Rectangle){ 25, 25, 125, 30 }, GuiIconText(ICON_FILE_OPEN, "Open serial port")))
                    serial_port = serial_port_open("/dev/ttyACM0");
            } else {
                if (GuiButton((Rectangle){ 25, 25, 125, 30 }, GuiIconText(ICON_FILE_OPEN, "Close serial port"))) {
                    serial_port = serial_port_close(serial_port);
                }
            }
            
            
            if(serial_port != -1) {
                char read_buffer[256];
                int num_bytes = read(serial_port, &read_buffer, sizeof(read_buffer));
                if(num_bytes < 0) {
                    perror("read");
                    close(serial_port);
                    serial_port = -1;
                } else {
                    char *n = strtok(read_buffer, "\n");
                    do {
                        graph[gptr] = atoi(n);
                        gptr = (gptr + 1) % WIDTH;
                    } while (n = strtok(NULL, " "));
                }
            }

            for(int x1 = 0; x1 < WIDTH - 1; x1++) {
                const int margin = 10;
                int x2 = x1 + 1;
                int y1 = HEIGHT - margin - graph[(gptr + x1) % WIDTH];
                int y2 = HEIGHT - margin - graph[(gptr + x2) % WIDTH];
                DrawLine(x1, y1, x2, y2, RED);
            }


        EndDrawing();
    }
    CloseWindow();

    return 0;
}