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

#include <assert.h>
#include <errno.h>
#include <fcntl.h> 
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "pioneer_p3dx_pvt.h"

// ============================================================================
//  Pioneer Basic
// ============================================================================

int pioneer_read_block(pioneer_t* pioneer, const uint8_t count, uint8_t* buffer) {
    int rest = count;
    int wrte = 0;
    int tried = 0;
    do {
        const int size = read(pioneer->tty.fd, &buffer[wrte], rest);
        if ( size == 0 ) {
            tty_select(&pioneer->tty, 10);
            tried += 1;
            if ( tried > 3 ) {
                return 0;
            }
        } else {
            rest -= size;
            wrte += size;
        }
    } while ( rest > 0 );
    return wrte;
}

int pioneer_read(pioneer_t* pioneer) {
    // read data from serial port
    int pack_size = 0;
    uint8_t buffer[PACK_MAX_SIZE];
    int tried = 0;

    int state = 0;
    for(int i=0; i<3;) {
        uint8_t c = 0;
        const int size = read(pioneer->tty.fd, &c, 1);
        if ( size == 0 ) {
            tty_select(&pioneer->tty, 25);
            tried += 1;
            if ( tried > 20 ) {
                printf("Nenhum dado recebido");
                exit(1);
            }
            continue;
        } else {
            tried = 0;
        }

        // waiting for 0xFA
        if ( state == 0 ) {
            if ( c == 0xFA ) {
                buffer[0] = c;
                state = 1;
            }
            i += 1;

        // waiting for 0xFB
        } else if ( state == 1 ) {
            if ( c == 0xFB ) {
                buffer[1] = c;
                state = 2;
            } else {
                state = 0;
            }

        // read the size of package
        } else if ( state == 2 ) {
            buffer[2] = c;
            const uint8_t count = c;
            const uint8_t size = pioneer_read_block(pioneer, count, &buffer[3]);
            if ( size == 0 ) {
                printf("error 1\r\n");
                return 0;
            }
            pack_size = 3 + size;
            break;
        }
    }

    // Success
    if ( pack_size > 3 ) {
        uint8_t cmd = buffer[3];
        if ( cmd == 0x32 || cmd == 0x33 ) {
            pioneer->pos_x = *( (int16_t*) &buffer[4] );
            pioneer->pos_y = *( (int16_t*) &buffer[6] );
        }
    }

    printf("recv: ");
    for (int i=0; i<pack_size; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\r\n");

    printf("- x: %d, y: %d \r\n", pioneer->pos_x, pioneer->pos_y);
    return pack_size;
}


int pioneer_exec(pioneer_t* pioneer, uint8_t command) {
    pack_init(pioneer->pack_send, command);
    pack_finish(pioneer->pack_send);
    tty_send_pack(&pioneer->tty, pioneer->pack_send);
    return OK;
}

int pioneer_exec_with_answer(pioneer_t* pioneer, uint8_t command) {
    pack_init(pioneer->pack_send, command);
    pack_finish(pioneer->pack_send);
    tty_send_pack(&pioneer->tty, pioneer->pack_send);
    pioneer_read(pioneer);
    return OK;
}

int pioneer_exec_va(pioneer_t* pioneer, uint8_t command, char* format, ...) {
    // init package
    va_list list;
    va_start(list, format);
    pack_init(pioneer->pack_send, command);

    // put the variable
    for (int i=0; i<5; i++) {
        const char c = format[i];
        if ( c == '\0' || c == '\n' ) {
            break;
        }
        if ( c == 'i' ) {
            int16_t val = (int16_t) va_arg(list, int32_t);
            pack_put_i16(pioneer->pack_send, val);
        } else if ( c == 'u' ) {
            uint16_t val = (uint16_t) va_arg(list, int32_t);
            pack_put_u16(pioneer->pack_send, val);
        }
    }

    va_end(list);

    // put checksum,  send and receive
    pack_finish(pioneer->pack_send);
    tty_send_pack(&pioneer->tty, pioneer->pack_send);

    // success
    return OK;
}

// ============================================================================
//  Pioneer Commands
// ============================================================================

/**
 * Inicializa o objeto da classe pioneer. Apenas zera as variaveis e abre a
 * conexao com o dispositivo serial /dev/ttyUSB0 com a velocidade 9600
 */

int pioneer_init(pioneer_t* pioneer) {
    pioneer->buf_recv_wrte0 = 0;
    pioneer->buf_recv_wrte1 = 0;
    pioneer->pack_recv = (pack_t*) &pioneer->buf_recv[0];
    pioneer->pack_send = (pack_t*) &pioneer->buf_send[0];
    pioneer->pos_x = 0;
    pioneer->pos_y = 0; 
    tty_init(&pioneer->tty, B9600);
    return OK;
}


/**
 * Envia comandos para se conectar com o robo
 * 
 * @brief
 *          Ass     | Count | Command | Checksum
 * -----------------------------------------------
 * sent1: 0xFA 0xFB | 0x03  | 0x00    | 0x00 0x00
 * recv1: 0xFA 0xFB | 0x03  | 0x00    | 0x00 0x00
 * 
 * sent2: 0xFA 0xFB | 0x03  | 0x01    | 0x00 0x01
 * recv2: 0xFA 0xFB | 0x03  | 0x01    | 0x00 0x01
 * 
 * sent3: 0xFA 0xFB | 0x03  | 0x02    | 0x00 0x02
 * recv3: 0xFA 0xFB | 0x03  | 0x02    | 0x00 0x02
 * 
 * sent4: 0xFA 0xFB | 0x03  | 0x01    | 0x00 0x01
 * recv4:
 *
 * @return error code
 */

int pioneer_connect(pioneer_t* pioneer) {
    // Sync0,1,2
    assert( pioneer_exec_with_answer(pioneer, 0x00) == OK );
    assert( pioneer_exec_with_answer(pioneer, 0x01) == OK );
    assert( pioneer_exec_with_answer(pioneer, 0x02) == OK );

    // Open server
    assert(pioneer_exec_with_answer(pioneer, 0x01) == OK );

    // Success
    return OK;
}

/**
 * Fecha conexao com o robo
 * 
 * @brief
 *          Ass    | Count | Command | Checksum
 * -----------------------------------------------
 * sent: 0xFA 0xFB | 0x03  | 0x02    | 0x00 0x02
 *
 * @return error code
 */
int pioneer_close(pioneer_t* pioneer) {
    return pioneer_exec(pioneer, 0x02);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | Checksum
 * -----------------------------------------------
 * sent: 0xFA 0xFB | 0x03  | 0x00    | 0x00 0x00
 *
 * @return error code
 */
int pioneer_pulse(pioneer_t* pioneer) {
    pioneer_exec(pioneer, 0x00);
    // pioneer_read(&pioneer);
    return 0;
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1      | ArgValue1 | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 28      | 0x1B (uint16) | 0x01 0x00 | 0x00 0x00
 *
 * @return error code
 */
int pioneer_enable_sonars(pioneer_t* pioneer) {
    return pioneer_exec_va(pioneer, 28, "u", 1);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1      | ArgValue1 | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 28      | 0x1B (uint16) | 0x00 0x00 | 0x00 0x00
 *
 * @return error code
 */
int pioneer_disable_sonars(pioneer_t* pioneer) {
    return pioneer_exec_va(pioneer, 28, "u", 0);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1      | ArgValue1 | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 4       | 0x1B (uint16) | 0x01 0x00 | 0x00 0x00
 *
 * @return error code
 */
int pioneer_enable_motors(pioneer_t* pioneer) {
    return pioneer_exec_va(pioneer, 4, "u", 1);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1      | ArgValue1 | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 4       | 0x1B (uint16) | 0x00 0x00 | 0x00 0x00
 *
 * @return error code
 */
int pioneer_disable_motors(pioneer_t* pioneer) {
    return pioneer_exec_va(pioneer, 4, "u", 0);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1     | ArgValue1     | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 11      | 0x3B (int16) | vel[0] vel[1] | 0x?? 0x??
 *
 * @return error code
 */
int pioneer_vel(pioneer_t* pioneer, int16_t vel) {
    return pioneer_exec_va(pioneer, 11, "i", vel);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1     | ArgValue1     | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 32      | 0x3B (int16) | vel1 vel2     | 0x?? 0x??
 *
 * @return error code
 */
int pioneer_vel2(pioneer_t* pioneer, int8_t vel1, int8_t vel2) {
    uint16_t vel;
    vel = vel1;
    vel += ((uint16_t) vel2) << 8;
    return pioneer_exec_va(pioneer, 32, "i", vel);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1     | ArgValue1               | Checksum
 * ---------------------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 21      | 0x3B (int16) | rotvel[0] rotval[1]     | 0x?? 0x??
 *
 * @return error code
 */
int pioneer_rotvel(pioneer_t* pioneer, int16_t rotvel) {
    return pioneer_exec_va(pioneer, 21, "i", rotvel);
}
