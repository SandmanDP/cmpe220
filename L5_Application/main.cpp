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
#include "ansotas/gps.h"
#include "lpc_pwm.hpp"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "semphr.h"

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

typedef struct {
        uint8_t length;
        uint8_t resp_id;
} gpspackage_t;

typedef struct {
        uint32_t lat;
        uint32_t lon;
} waypoint_t;

waypoint_t list[7] = {
        {37043315,121405522},
        {37043232,121405241},
        {37043277,121404951},
        {37043060,121405029},
        {37043429,121405367},
        {37043223,121405479},
        {37043275,121405617},
};

uint32_t compass_fix_list[36] = {
        212,
        370,
        562,
        718,
        813,
        925,
        1044,
        1101,
        1180,
        1244,
        1327,
        1377,
        1442,
        1516,
        1572,
        1630,
        1691,
        1762,
        1835,
        1860,
        1916,
        1989,
        2052,
        2135,
        2183,
        2267,
        2371,
        2465,
        2581,
        2660,
        2850,
        2976,
        3135,
        3270,
        3436,
        72,
};

uint32_t compass_fixifier(uint32_t input){
    int tempval = ((input+50)/100)%36;
    return compass_fix_list[tempval];
}

int32_t offset_lat = 0;
int32_t offset_lon = 0;

int32_t distance = 0, angle = 0;

SemaphoreHandle_t sema;

//const gpspackage_t gps_config[256] =
//{
//        [sys_restart] = { .length = 0, .resp_id = 0},
//};

//static char gps_string[256];

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
            int compass = 0;
//            LPC_UART3->THR = 0x13;                  //send byte "i" from the message string
//            while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
//            LPC_UART3->THR = 0xF0;                  //send byte "i" from the message string
//            while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
//            while(!(LPC_UART3->LSR & (1 << 0)));
//            int temp1 = LPC_UART3->RBR;
//            LPC_UART3->THR = 0xF5;                  //send byte "i" from the message string
//            while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
//            while(!(LPC_UART3->LSR & (1 << 0)));
//            int temp2 = LPC_UART3->RBR;
//            LPC_UART3->THR = 0xF6;                  //send byte "i" from the message string
//            while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
//            while(!(LPC_UART3->LSR & (1 << 0)));
//            int temp3 = LPC_UART3->RBR;
//            printf("%x %x %x", temp1, temp2, temp3);
//            while(!BIT(LPC_GPIO1->FIOPIN).b9);
//            LPC_UART3->THR = 0xF8;                  //send byte "i" from the message string
//            while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
//            while(!(LPC_UART3->LSR & (1 << 0)));
//            printf("Initial calibration.");
            LPC_UART3->THR = 0x13;                  //send byte "i" from the message string
            while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
            while(!(LPC_UART3->LSR & (1 << 0)));
            compass += LPC_UART3->RBR*256;
            while(!(LPC_UART3->LSR & (1 << 0)));
            compass += LPC_UART3->RBR;
            int default_position = compass;
            while(1){
                vTaskDelay(100);
                compass = 0;
                LPC_UART3->THR = 0x13;                  //send byte "i" from the message string
                while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
                while(!(LPC_UART3->LSR & (1 << 0)));
                compass += LPC_UART3->RBR*256;
                while(!(LPC_UART3->LSR & (1 << 0)));
                compass += LPC_UART3->RBR;
//                float temp = ((float)compass+1800-default_position)/240.0;
//                temp = temp >= 5.0 && temp <= 10.0 ? temp : (temp >= 5.0 ? 10 : 5);
                //pwm.set(temp);
                xSemaphoreTake(sema, 10000);
                printf("{\"direction\": %i}\n", compass/10);
                xSemaphoreGive(sema);
//                printf("Compass: %i; Adjusted: %f\n", compass, temp);
                //pwm.set((compass%900)/180.0f+5);
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
//            while(1){
//                printf("{\"temperature\": 70, \"direction\": 180, \"gps_lat\": 37.336, \"gps_long\": -121.88}\n");
//                vTaskDelay(500);
//            }
            while (1) {
                gps_send_message(sys_config_nmea, 0);

                //printf("\n%s: %i %i %i\n", result ? "True" : "False", data[0], data[1], data[2]);

                gps_send_message(gps_config_datum, 0);

                gps_send_message(sys_config_nmea, 1);

                uint8_t data3[256] = {0};
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
//                        printf("%s", data3);
                        xSemaphoreTake(sema, 10000);
                        printf("{\"gps_lat\": %f, \"gps_long\": -%f}\n", latd+(latm*5.0/3000000), longd+(longm*5.0/3000000));
                        xSemaphoreGive(sema);
                    }
                    vTaskDelay(900);
//                    if(l == 0){
//                        if(up==0){
//                            ave_ns = 0;
//                            ave_ew = 0;
//                        }else if(up>=60){
//                            offset_lat = (ave_ns/30)-list[0].lat;
//                            offset_lon = (ave_ew/30)-list[0].lon;
//                            printf("Done calib..: off_lan = %i, off_lon = %i, ave_ns = %i, ave_ew = %i", (int)offset_lat, (int)offset_lon, (int)ave_ns, (int)ave_ew);
//                            l++;
//                        }else if(up>=30){
//                            ave_ns += ns;
//                            ave_ew += ew;
//                        }
//                    }
//                    else {
//                        act_ns = ns-offset_lat;
//                        act_ew = ew-offset_lon;
//                        distance = (int)sqrt(pow((act_ns-list[l].lat),2)+pow((act_ew-list[l].lon),2));
//                        if(distance <= 100){
//                            l++;
//                            if(l < 7){
//                                distance = (int)sqrt(pow((act_ns-list[l].lat),2)+pow((act_ew-list[l].lon),2));
//                            }
//                            else{
//                                break;
//                            }
//                        }
//                        printf("\n%i: act_ns:%i act_ew:%i, distance:%i", l, (int)act_ns, (int)act_ew, (int)distance);
//                        printf("Done calib..: off_lan = %i, off_lon = %i, ave_ns = %i, ave_ew = %i", (int)offset_lat, (int)offset_lon, (int)ave_ns, (int)ave_ew);
//                        angle = atan2(list[l].lat-act_ns,act_ew-list[l].lon)*1800/M_PI;
//                        angle = compass_fixifier((3600 - ((angle+6300)%3600))%3600);
//                        int compass = 0;
//                        LPC_UART3->THR = 0x13;                  //send byte "i" from the message string
//                        while(!(LPC_UART3->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                        while(!(LPC_UART3->LSR & (1 << 0)));
//                        compass += LPC_UART3->RBR*256;
//                        while(!(LPC_UART3->LSR & (1 << 0)));
//                        compass += LPC_UART3->RBR;
//                        //int raw = compass;
//                        //compass = (3600 - ((compass+2700)%3600))%3600;
//                        int angle_diff = abs(angle-compass)%3600;
//                        angle_diff = angle_diff > 1800 ? 3600 - angle_diff : angle_diff;
//                        angle_diff *= ((angle-compass) >= 0 && (angle-compass) <= 1800) || ((angle-compass) <=-1800) ? -1 : 1;
//                        printf("\nCompass: %i, Wanted: %i, Diff: %i", (int)compass, (int)angle, angle_diff);
//                    }
//                    printf("\n%i,N %i,W (%i)", (int)ns-70, (int)ew+550, (int)up);
//                    for(int j = 0; j < i+1; j++){
//                        printf("%c", data3[j]);
//                    }
                    data3[0] = 0;
                }
                while(1);


                uint8_t data[7] = { 0 };
                bool result = gps_send_message(gps_get_datum, 0);

                for (int i = 0; i < 7; i++) {
                    while(!(LPC_UART2->LSR & (1 << 0)));
                    data[i] = LPC_UART2->RBR;
                }

                uint8_t payload = data[6];

                uint8_t data2[56];
                for(int i = 0; i < payload + 3; i++){
                    while(!(LPC_UART2->LSR & (1 << 0)));
                    data2[i] = LPC_UART2->RBR;
                }

                printf("\n");
                for (int i = 0; i < 7; i++) {
                    printf("%i\n", data[i]);
                }

                for (int i = 0; i < payload + 3; i++) {
                    printf("%i\n", data2[i]);
                }

                data2[1] = result;

                while(1);
//                while(!(LPC_UART2->LSR & (1 << 0)));
//                uint8_t char_save = LPC_UART2->RBR;
//                int count = 0;
//                while (char_save == '$') {
//                    printf("Received message");
//                    while(!(LPC_UART2->LSR & (1 << 0)));
//                    uint8_t char_save2 = LPC_UART2->RBR;
//                    while (char_save == '*') {
//                        while(!(LPC_UART2->LSR & (1 << 0)));
//                        uint8_t char_save3 = LPC_UART2->RBR;
//                        while(!(LPC_UART2->LSR & (1 << 0)));
//                        char_save3 = LPC_UART2->RBR;
//                        while(!(LPC_UART2->LSR & (1 << 0)));
//                        char_save3 = LPC_UART2->RBR;
//                        while(!(LPC_UART2->LSR & (1 << 0)));
//                        char_save3 = LPC_UART2->RBR;
//                        char_save2 = char_save3;
//                        char_save3 = char_save2;
//                    }
//                    if(++count == 5) break;
//                }
//                LPC_UART2->THR = 0x08;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                LPC_UART2->THR = 0;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                LPC_UART2->THR = 0;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                LPC_UART2->THR = 0;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                LPC_UART2->THR = 0;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                LPC_UART2->THR = 0;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                LPC_UART2->THR = 0;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                LPC_UART2->THR = 0;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                LPC_UART2->THR = 0;                  //send byte "i" from the message string
//                while(!(LPC_UART2->LSR & (1 << 5)));    //wait for empty transmitter holding register
//                printf("Done disabling");
            }

//            while(1){
//                for(int i = 0; i < 255; i++) {
//                    while(!(LPC_UART2->LSR & (1 << 0)));
//                    uint8_t char_save = LPC_UART2->RBR;
//                    if (char_save == '$') char_save = '-';
//                    gps_string[i] = char_save;
//                }
//                gps_string[255] = '\0';
//                printf("%s\n", gps_string);
//            }
            return true;
        }
};

int main(void)
{
//    /**
//     * A few basic tasks for this bare-bone system :
//     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
//     *      2.  Remote task allows you to use remote control to interact with the board.
//     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
//     *
//     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
//     * such that it can save remote control codes to non-volatile memory.  IR remote
//     * control codes can be learned by typing the "learn" terminal command.
//     */
//    scheduler_add_task(new terminalTask(PRIORITY_HIGH));
//
//    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
//    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
//
//    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
//    #if 0
//    const bool run_1Khz = false;
//    scheduler_add_task(new periodicSchedulerTask(run_1Khz));
//    #endif
//
//    /* The task for the IR receiver to "learn" IR codes */
//    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));
//
//    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
//     * task to always be responsive so you can poke around in case something goes wrong.
//     */
//
//    /**
//     * This is a the board demonstration task that can be used to test the board.
//     * This also shows you how to send a wireless packets to other boards.
//     */
//    #if 0
//        scheduler_add_task(new example_io_demo());
//    #endif
//
//    /**
//     * Change "#if 0" to "#if 1" to enable examples.
//     * Try these examples one at a time.
//     */
//    #if 0
//        scheduler_add_task(new example_task());
//        scheduler_add_task(new example_alarm());
//        scheduler_add_task(new example_logger_qset());
//        scheduler_add_task(new example_nv_vars());
//    #endif
//
//    /**
//	 * Try the rx / tx tasks together to see how they queue data to each other.
//	 */
//    #if 0
//        scheduler_add_task(new queue_tx());
//        scheduler_add_task(new queue_rx());
//    #endif
//
//    /**
//     * Another example of shared handles and producer/consumer using a queue.
//     * In this example, producer will produce as fast as the consumer can consume.
//     */
//    #if 0
//        scheduler_add_task(new producer());
//        scheduler_add_task(new consumer());
//    #endif
//
//    /**
//     * If you have RN-XV on your board, you can connect to Wifi using this task.
//     * This does two things for us:
//     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
//     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
//     *
//     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
//     * @code
//     *     // Assuming Wifly is on Uart3
//     *     addCommandChannel(Uart3::getInstance(), false);
//     * @endcode
//     */
//    #if 0
//        Uart3 &u3 = Uart3::getInstance();
//        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
//        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
//    #endif
	//Set PWM Pin 2.0
	LPC_PINCON -> PINSEL4 &= ~(15<<0);
	LPC_PINCON -> PINSEL4 |= (5<<0);

    //scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    scheduler_add_task(new compass_task(PRIORITY_HIGH));
    scheduler_add_task(new gps_task(PRIORITY_HIGH));
    sema = xSemaphoreCreateBinary();

    scheduler_start(); ///< This shouldn't return
    return -1;
}
