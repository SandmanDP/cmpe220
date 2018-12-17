/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */


#include "tasks.hpp"
#include "examples/examples.hpp"
#include "LPC17xx.h"
#include "lpc_pwm.hpp"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "semphr.h"
#include <string.h>
#include "wireless.h"

typedef enum {
    sys_restart = 1,
    sys_software_ver = 2,
    sys_software_crc = 3,
    sys_factory_reset = 4,
    sys_serial_config = 5,
    sys_config_nmea = 8,
    sys_config_msg_fmt = 9,
    sys_config_pwr_mode = 12,
    sys_config_update_rate = 14,
    sys_get_update_rate = 16,

    gps_config_datum = 41,
    gps_get_datum = 45,
    gps_get_ephemeris = 48,
    gps_set_ephemeris = 49,
    gps_config_waas = 55,
    gps_get_waas = 56,
    gps_config_pos_pin = 57,
    gps_get_pos_pin = 58,
    gps_config_pin_param = 59,
    gps_config_nav_mode = 60,
    gps_get_nav_mode = 61,
    gps_config_1pps_mode = 62,
    gps_get_1pps_mode = 63,

} gpsmessage_t;

typedef enum {
    compass = 1,
    gps = 2,
    temp = 3,
    accel = 4,
    carbon = 5,
} sensor_t;

int32_t offset_lat = 0;
int32_t offset_lon = 0;

SemaphoreHandle_t sema;

bool gps_send_message(gpsmessage_t msg, uint8_t param)
{
    uint8_t message_data[100] = { 0xA0, 0xA1, 0 };
    uint8_t cs = msg;
    uint16_t length = 0;

    switch (msg) {
        case sys_restart:
            length = 15;
            break;
        case sys_software_ver:
            length = 2;
            break;
        case sys_software_crc:
            length = 2;
            break;
        case sys_factory_reset:
            length = 2;
            break;
        case sys_serial_config:
            length = 4;
            break;
        case sys_config_nmea:
            length = 9;
            message_data[5] = 0;
            message_data[6] = 0;
            message_data[7] = 0;
            message_data[8] = param;
            message_data[9] = 0;
            message_data[10] = 0;
            message_data[11] = 0;
            message_data[12] = 0;
            break;

        case gps_config_datum:
            length = 19;
            message_data[5] = 0;
            message_data[6] = 134;
            message_data[7] = 6;
            message_data[8] = 255;
            message_data[9] = 250;
            message_data[10] = 0;
            message_data[11] = 159;
            message_data[12] = 0;
            message_data[13] = 175;
            message_data[14] = 0x00;
            message_data[15] = 0x7D;
            message_data[16] = 0x38;
            message_data[17] = 0x40;
            message_data[18] = 0x01;
            message_data[19] = 0x2D;
            message_data[20] = 0xEC;
            message_data[21] = 0xE6;
            message_data[22] = 0;
            break;
        case gps_get_datum:
            length = 1;
            break;

        case gps_set_ephemeris:
            length = 87;
            break;

        default:
            break;
    }

    for (int index = 5; index < length+4; index++) {
        cs = cs ^ message_data[index];
    }

    message_data[2] = length/256;
    message_data[3] = length%256;
    message_data[4] = msg;
    message_data[4+length] = cs;
    message_data[5+length] = 0x0D;
    message_data[6+length] = 0x0A;

    for (int i = 0; i < 7+length; i++) {
        LPC_UART2->THR = message_data[i];       //send byte "i" from the message string
        while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
    }

    uint8_t char_save1 = 0, char_save2 = 0;

    while (!(char_save1 == 131 || char_save1 == 132) || char_save2 != msg) {
        while (!(char_save1 == 131 || char_save1 == 132)) {
            while(!(LPC_UART2->LSR & (1 << 0)));
            char_save1 = LPC_UART2->RBR;
        }
        while(!(LPC_UART2->LSR & (1 << 0)));
        char_save2 = LPC_UART2->RBR;
    }

    return char_save1 == 131;
}

class compass_task : public scheduler_task {
    public:

        compass_task(uint8_t priority) : scheduler_task("compass", 512*4, priority){}

        bool init(void){
            BIT(LPC_SC->PCONP).b25 = 1;                     //UART3 power enable
            BIT(LPC_SC->PCLKSEL1).b19_18 = 1;               //UART3 peripheral clock selection (PCLK_peripheral = CCLK)
            BIT(LPC_PINCON->PINSEL9).b25_24 = 3;            //TXD3 function
            BIT(LPC_PINCON->PINSEL9).b27_26 = 3;            //RXD3 function
            uint16_t DL = sys_get_cpu_clock()/(9600*16);    //divisor latch value
            BIT(LPC_UART3->LCR).b7 = 1;                     //set DLAB to one to change baud rate divisor value
            LPC_UART3->DLM = DL>>8;                         //set 8 most significant bits
            LPC_UART3->DLL = DL>>0;                         //set 8 least significant bits
            BIT(LPC_UART3->LCR).b1_0 = 3;                   //set 8-bit character word length
            BIT(LPC_UART3->LCR).b7 = 0;                     /*set DLAB back to zero to read from the receiver buffer
                                                            register and write to the transmit holding register*/
            return true;
        }

        bool run(void *p){
            uint16_t comp = 0;
            char data2[24] = {0};
            LPC_UART3->THR = 0x13;                  //send byte "i" from the message string
            while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
            while(!(LPC_UART3->LSR & (1 << 0)));
            comp += LPC_UART3->RBR*256;
            while(!(LPC_UART3->LSR & (1 << 0)));
            comp += LPC_UART3->RBR;
            while(1){
                vTaskDelay(100);
                comp = 0;
                LPC_UART3->THR = 0x13;                  //send byte "i" from the message string
                while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
                while(!(LPC_UART3->LSR & (1 << 0)));
                comp += LPC_UART3->RBR*256;
                while(!(LPC_UART3->LSR & (1 << 0)));
                comp += LPC_UART3->RBR;
                data2[0] = compass;
                data2[1] = comp%256;
                data2[2] = comp/256;
                wireless_send(MESH_BROADCAST_ADDR, mesh_pkt_nack, data2, 3, 1);
//                size = sprintf(data2, "{\"direction\": %i}\n", compass/10);
//                xSemaphoreTake(sema, 10000);
//                wireless_send(MESH_BROADCAST_ADDR, mesh_pkt_nack, data2, size, 3);
////                printf("{\"direction\": %i}\n", compass/10);
//                xSemaphoreGive(sema);
            }
            return true;
        }
};

class gps_task : public scheduler_task {
    public:

        gps_task(uint8_t priority) : scheduler_task("gps", 512*8, priority){}

        bool init(void){
            BIT(LPC_SC->PCONP).b24 = 1;                     //UART2 power enable
            BIT(LPC_SC->PCLKSEL1).b17_16 = 1;               //UART2 peripheral clock selection (PCLK_peripheral = CCLK)
            BIT(LPC_PINCON->PINSEL4).b17_16 = 2;            //TXD2 function
            BIT(LPC_PINCON->PINSEL4).b19_18 = 2;            //RXD2 function
            uint16_t DL = sys_get_cpu_clock()/(9600*16);    //divisor latch value
            BIT(LPC_UART2->LCR).b7 = 1;                     //set DLAB to one to change baud rate divisor value
            LPC_UART2->DLM = DL>>8;                         //set 8 most significant bits
            LPC_UART2->DLL = DL>>0;                         //set 8 least significant bits
            BIT(LPC_UART2->LCR).b1_0 = 3;                   //set 8-bit character word length
            BIT(LPC_UART2->LCR).b7 = 0;                     /*set DLAB back to zero to read from the receiver buffer
                                                            register and write to the transmit holding register*/

            BIT(LPC_SC->PCONP).b25 = 1;                     //UART3 power enable
            BIT(LPC_SC->PCLKSEL1).b19_18 = 1;               //UART3 peripheral clock selection (PCLK_peripheral = CCLK)
            BIT(LPC_PINCON->PINSEL9).b25_24 = 3;            //TXD3 function
            BIT(LPC_PINCON->PINSEL9).b27_26 = 3;            //RXD3 function
            DL = sys_get_cpu_clock()/(9600*16);    //divisor latch value
            BIT(LPC_UART3->LCR).b7 = 1;                     //set DLAB to one to change baud rate divisor value
            LPC_UART3->DLM = DL>>8;                         //set 8 most significant bits
            LPC_UART3->DLL = DL>>0;                         //set 8 least significant bits
            BIT(LPC_UART3->LCR).b1_0 = 3;                   //set 8-bit character word length
            BIT(LPC_UART3->LCR).b7 = 0;                     /*set DLAB back to zero to read from the receiver buffer
                                                            register and write to the transmit holding register*/
            return true;
        }

        bool run(void *p){
            gps_send_message(sys_config_nmea, 0);
            gps_send_message(gps_config_datum, 0);
            gps_send_message(sys_config_nmea, 1);

            char data3[256] = {0};
//            uint8_t size = 0;
            uint32_t latd = 0, latm = 0, longd = 0, longm = 0;
            while(1){
                while(data3[0] != '$') {
                    while(!(LPC_UART2->LSR & (1 << 0)));
                    data3[0] = LPC_UART2->RBR;
                }
                int i = 1;
                for(; ; i++){
                    while(!(LPC_UART2->LSR & (1 << 0)));
                    data3[i] = LPC_UART2->RBR;
                    if(data3[i-1] == 13 && data3[i] == 10) break;
                }
                latd = data3[7]-'0';
                latd = latd*10 + data3[8]-'0';
                latm = data3[9]-'0';
                latm = latm*10 + data3[10]-'0';
                latm = latm*10 + data3[12]-'0';
                latm = latm*10 + data3[13]-'0';
                latm = latm*10 + data3[14]-'0';
                latm = latm*10 + data3[15]-'0';
                longd = data3[19]-'0';
                longd = longd*10 + data3[20]-'0';
                longd = longd*10 + data3[21]-'0';
                longm = data3[22]-'0';
                longm = longm*10 + data3[23]-'0';
                longm = longm*10 + data3[25]-'0';
                longm = longm*10 + data3[26]-'0';
                longm = longm*10 + data3[27]-'0';
                longm = longm*10 + data3[28]-'0';
                if(data3[43] == 'A') {
                    data3[0] = gps;
                    data3[1] = latd%256;
                    memcpy(&data3[2], &latm, 4);
                    data3[6] = longd%256;
                    memcpy(&data3[7], &longm, 4);
                    wireless_send(MESH_BROADCAST_ADDR, mesh_pkt_nack, data3, 11, 1);
//                    size = sprintf(data3, "{\"gps_lat\": %f, \"gps_long\": -%f}\n", latd+(latm*5.0/3000000), longd+(longm*5.0/3000000));
//                    xSemaphoreTake(sema, 10000);
//                    wireless_send(MESH_BROADCAST_ADDR, mesh_pkt_nack, data3, size, 3);
////                    printf("{\"gps_lat\": %f, \"gps_long\": -%f}\n", latd+(latm*5.0/3000000), longd+(longm*5.0/3000000));
//                    xSemaphoreGive(sema);
                }
                vTaskDelay(900);
                data3[0] = 0;
            }

            return true;
        }
};

//class serial_task : public scheduler_task {
//    public:
//
//        serial_task(uint8_t priority) : scheduler_task("serial", 512*4, priority){}
//
//        bool init(void){
//            return true;
//        }
//
//        bool run(void *p){
//            while(1){
//                wireless_send(MESH_BROADCAST_ADDR, mesh_pkt_nack, "Hello!", 6, 1);
//                vTaskDelay(900);
////                if (wireless_get_rx_pkt(&pkt, 10000))
////                    printf((char*)pkt.data);
//            }
//        }
//};

int main(void)
{
	//Set PWM Pin 2.0
//	LPC_PINCON -> PINSEL4 &= ~(15<<0);
//	LPC_PINCON -> PINSEL4 |= (5<<0);

    //scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    scheduler_add_task(new compass_task(PRIORITY_MEDIUM));
    scheduler_add_task(new gps_task(PRIORITY_MEDIUM));
//    scheduler_add_task(new serial_task(PRIORITY_LOW));
    scheduler_add_task(new wirelessTask(PRIORITY_HIGH));
    sema = xSemaphoreCreateBinary();

    scheduler_start(); ///< This shouldn't return
    return -1;
}
