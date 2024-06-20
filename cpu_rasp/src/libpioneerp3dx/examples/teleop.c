/**
 * Pioneer-P3DX communication library
 *  Copyright (C) 2024  Visao Robotica e Imagem (VRI)
 *   - Felipe Bombardelli <felipebombardelli@gmail.com>
 * 
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * * */

// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

#include "pioneer_p3dx.h"

struct termios orig_termios;

// ============================================================================
//  Private Function for TTY
// ============================================================================

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
    struct timeval tv = { 0L, 50*1000L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv) > 0;
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

// ============================================================================
//  Main
// ============================================================================

int main(int argc, char *argv[]) {
    // Begin
    set_conio_terminal_mode();
    pioneer_t pioneer;
    pioneer_init(&pioneer);
    pioneer_connect(&pioneer);
    pioneer_disable_sonars(&pioneer);
    pioneer_enable_motors(&pioneer);

    // Main loop
    printf("Main loop\r\n");
    while(1) {
        if ( kbhit() ) {
            const char c = getch();
            if ( c == 'q' ) {
                break;
            } else if ( c == 'a' ) {
                pioneer_rotvel(&pioneer, 15);
            } else if ( c == 'd' ) {
                pioneer_rotvel(&pioneer, -15);
            } else if ( c == 'w' ) {
                // pioneer_vel(&pioneer, 50);
                pioneer_vel2(&pioneer, 5, 5);
            } else if ( c == 's' ) {
                pioneer_vel(&pioneer, -50);
            } else {
                // printf("%c\n", c);
            }
        }

        pioneer_pulse(&pioneer);
        pioneer_read(&pioneer);
        // usleep(10000);
    }

    // End
    pioneer_close(&pioneer);
    return 0;
}