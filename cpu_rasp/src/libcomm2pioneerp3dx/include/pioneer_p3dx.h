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

#include <stdint.h>

#define PACK_MAX_SIZE 256

#define OK      0
#define ERROR   1

// ============================================================================
//  Private Strutures
// ============================================================================

typedef struct {
    uint8_t header[2];
    uint8_t count;
    uint8_t command;
    uint8_t argdata[];
} pack_t;

typedef struct {
    int state;
    int fd;
} tty_t;

typedef struct {
    uint8_t buf_send[PACK_MAX_SIZE];
    uint8_t buf_recv[2][PACK_MAX_SIZE];
    int buf_recv_wrte0;
    int buf_recv_wrte1;
    pack_t* pack_send;
    pack_t* pack_recv;
    tty_t tty;
    int16_t pos_x;
    int16_t pos_y;
} pioneer_t;

// ============================================================================
//  Public Functions
// ============================================================================

// Initialize the pioneer object
int pioneer_init(pioneer_t* pioneer);

// Connect and disconnect the Pioneer P3DX robot
int pioneer_connect(pioneer_t* pioneer);
int pioneer_close(pioneer_t* pioneer);
int pioneer_pulse(pioneer_t* pioneer);

// Disable and Enable Sonars
int pioneer_disable_sonars(pioneer_t* pioneer);
int pioneer_enable_sonars(pioneer_t* pioneer);

// Disable and Enable Motors
int pioneer_disable_motors(pioneer_t* pioneer);
int pioneer_enable_motors(pioneer_t* pioneer);

// Set speed
int pioneer_vel(pioneer_t* pioneer, int16_t vel);
int pioneer_vel2(pioneer_t* pioneer, int8_t vel1, int8_t vel2);
int pioneer_rotvel(pioneer_t* pioneer, int16_t rotvel);

// Read data from Pioneer
int pioneer_read(pioneer_t* pioneer);