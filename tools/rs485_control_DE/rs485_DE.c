#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <gpiod.h>
#include <pty.h>
#include <linux/serial.h>

#define BUFFER_SIZE 256
#define RS485_CONSUMER "RS485_program"

struct gpiod_chip *chip;
struct gpiod_line *line;

struct gpiod_chip *rs485_chip;
struct gpiod_line *rs485_line;

void setup_gpio(char *rs485_de_chip, int rs485_de_line, char *rs485_en_chip, int rs485_en_line)
{
    int ret;

    // open GPIO controller
    chip = gpiod_chip_open(rs485_de_chip);
    if (!chip) {
        fprintf(stderr, "gpiod_chip_open");
        exit(EXIT_FAILURE);
    }

    // request GPIO line
    line = gpiod_chip_get_line(chip, rs485_de_line);
    if (!line) {
        fprintf(stderr, "Failed to get GPIO line %d\n", rs485_de_line);
        gpiod_chip_close(chip);
        exit(EXIT_FAILURE);
    }

    // set GPIO as output
    ret = gpiod_line_request_output(line, RS485_CONSUMER, 0);
    if (ret < 0) {
        fprintf(stderr, "Failed to request GPIO line as output: %s\n", strerror(-ret));
        gpiod_chip_close(chip);
        exit(EXIT_FAILURE);
    }
    if (rs485_en_chip)
    {
        // open GPIO controller
        rs485_chip = gpiod_chip_open(rs485_en_chip);
        if (!rs485_chip) {
            fprintf(stderr, "gpiod_chip_open");
            return;
        }

        // request GPIO line
        rs485_line = gpiod_chip_get_line(rs485_chip, rs485_en_line);
        if (!rs485_line) {
            fprintf(stderr, "Failed to get GPIO line %d\n", rs485_de_line);
            gpiod_chip_close(rs485_chip);
            return;
        }

        // set GPIO as output
        ret = gpiod_line_request_output(rs485_line, RS485_CONSUMER, 0);
        if (ret < 0) {
            fprintf(stderr, "Failed to request GPIO line as output: %s\n", strerror(-ret));
            gpiod_chip_close(rs485_chip);
            return;
        }
        gpiod_line_set_value(rs485_line, 1);
    }
    
}

void toggle_gpio_high() {
    gpiod_line_set_value(line, 1);
}

void toggle_gpio_low() {
    gpiod_line_set_value(line, 0);
}


int main(int argc, char* argv[]) {
    int master_fd, slave_fd, tty_fd;
    char slave_name[32];
    char *tty_name;
    struct termios tty_termios;
    struct termios tty;
    char buffer[BUFFER_SIZE];
    ssize_t n;
    int ready;
    fd_set readfds, writefds;
    int rs485_de_line, rs485_en_line;
    char *rs485_de_chip, *rs485_en_chip , *rs485_dir;


    if(argc < 4) {
        printf("Usage: %s /dev/ttyAMA* DE_CHIP(/dev/gpiochip*)  DE_LINE [RS485_dir] [EN_CHIP] [EN_LINE]\n", argv[0]);
        return 1;
    }

    tty_name = argv[1];
    rs485_de_chip = argv[2];
    rs485_de_line = atoi(argv[3]);
    rs485_dir = (argc > 4) ? argv[4] : "/dev/ttyAMA10";
    rs485_en_chip = (argc > 6) ? argv[5] : NULL;
    rs485_en_line = (argc > 6) ? atoi(argv[6]) : -1;

    setup_gpio(rs485_de_chip, rs485_de_line, rs485_en_chip, rs485_en_line);

    // create pseudo terminal
    if (openpty(&master_fd, &slave_fd, slave_name, NULL, NULL) == -1) {
        perror("openpty");
        return 1;
    }
    
    // create symlink
    if(symlink(slave_name, rs485_dir) == -1) {
        if(errno != EEXIST) {
            perror("symlink");
            return 1;
        } else {
            if (unlink(rs485_dir) == -1) {
                perror("unlink");
                return 1;
            }
            if(symlink(slave_name, rs485_dir) == -1) {
                perror("symlink");
                return 1;
            }
        }
    }

    // 0666 permissions
    chmod(slave_name, 0666);

    printf("slave_name: %s new tty: %s\n", slave_name, rs485_dir);
    // open serial port
    tty_fd = open(tty_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (tty_fd < 0) {
        perror("Unable to open serial port");
        return 1;
    }
    // set rs485 mode
    struct serial_rs485 rs485conf = {0};
    rs485conf.flags |= SER_RS485_ENABLED;
    if (ioctl (tty_fd, TIOCSRS485, &rs485conf) < 0) {
        perror("ioctl");
        return 1;
    }

    // set serial port attributes
    tcgetattr(master_fd, &tty);
    cfmakeraw(&tty); // set raw mode
    tcsetattr(master_fd, TCSANOW, &tty);
    tcgetattr(tty_fd, &tty_termios);
    // set baud rate
    cfsetispeed(&tty_termios, cfgetispeed(&tty));
    cfsetospeed(&tty_termios, cfgetispeed(&tty));
    tcsetattr(tty_fd, TCSANOW, &tty_termios);
    // monitor file descriptors
    FD_ZERO(&readfds);

    while (1) {
        FD_SET(master_fd, &readfds);
        FD_SET(tty_fd, &readfds);
        int max_fd = (tty_fd > master_fd) ? tty_fd : master_fd;
        ready = select(max_fd + 2, &readfds, NULL, NULL, NULL);
        if (ready == -1) {
            perror("select");
            break;
        }

        if (FD_ISSET(master_fd, &readfds)) {
            toggle_gpio_high();
            tcgetattr(master_fd, &tty);
            tcgetattr(tty_fd, &tty_termios);
            // set baud rate
            cfsetispeed(&tty_termios, cfgetispeed(&tty));
            cfsetospeed(&tty_termios, cfgetispeed(&tty));
            tcsetattr(tty_fd, TCSANOW, &tty_termios);
            n = read(master_fd, buffer, BUFFER_SIZE - 1);
            if (n > 0) {
                buffer[n] = '\0'; // make sure string is null terminated
                // send data to serial port
                write(tty_fd, buffer, n);
            }
            toggle_gpio_low();
        }
        
        if (FD_ISSET(tty_fd, &readfds)) {
            tcgetattr(master_fd, &tty);
            tcgetattr(tty_fd, &tty_termios);
            // set baud rate
            cfsetispeed(&tty_termios, cfgetispeed(&tty));
            cfsetospeed(&tty_termios, cfgetispeed(&tty));
            tcsetattr(tty_fd, TCSANOW, &tty_termios);
            n = read(tty_fd, buffer, BUFFER_SIZE - 1);
            if (n > 0) {
                buffer[n] = '\0'; 
                // send data to pseudo terminal
                write(master_fd, buffer, n);
            }
        }
    }

    // close serial port
    close(master_fd);
    close(slave_fd);

    return 0;
}