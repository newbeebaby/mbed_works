#include "ThisThread.h"
#include <algorithm>
#include "mbed.h"
#include "vesc.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX
#include "c620/c620.h"
#define MAXIMUM_BUFFER_SIZE   32   
#include"Sender.h"
using namespace std::chrono;
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);

Ticker ticker;
int32_t can_baud = 1000000;
CAN can(PB_5,PB_6,can_baud);    //can2, for arduino
CAN can2(PA_11,PA_12,can_baud); //can1, for bldc
int can_id = 80;
int can_id2 =90;
volatile uint16_t LX_L;
volatile uint16_t RX_R;
volatile int8_t elva;
volatile int8_t trig;
volatile int speed = 10000;
volatile int dis;
const int sens = 100;
volatile int8_t mm;

DigitalIn upbutton(PC_13); // User Button
DigitalOut led(LED1);
bool state = false;
vesc _vesc1;
vesc _vesc2;
c620 _m3508;
DigitalOut fire(PB_3);
int convert(float x);

char var[]= {'1','2','3'};
// main() runs in its own thread in the OS
const char head[] = {'A'};
const char foot[] = {'B'};
int Is_digit(int speed);


int convert(float x){
    dis = x/sens;
    return dis;
}

int Is_digit(int speed){
        int digit = 1;
        while(speed > 10){
        speed = speed/10;
        digit++;
        }
        return digit;
}
int main(){

  //printf("main running\r\n");
  CANMessage msg;
  _vesc1.vesc_init(&can, can_baud);
  _vesc1.set_monitor_id(can_id); 
  _vesc2.vesc_init(&can, can_baud);
  _vesc2.set_monitor_id(can_id2); 
  
  _m3508.c620_init(&can2);
  _m3508.set_i_pid_param(0, 1.0, 0.000, 0.000000); // Torque PID W1
  _m3508.set_v_pid_param(0, 1.0, 0.000, 0.000);  // Velocity PID W1
 // _m3508.set_p_pid_param(0, 1.5, 0.0, 0.0);     // position PID W1
  
  // set the LP filter of the desire control
  _m3508.profile_velocity_CCW[0] = 7000;//Maximum is 12000 for c620 and 10000 for c610
  _m3508.profile_velocity_CW[0] = -7000;
  _m3508.profile_torque_CCW[0] = 8000; //Maximum is 16000 for c620 and 10000 for c610
  _m3508.profile_torque_CW[0] = -8000;
  // set the current limit, overcurrent may destory the driver
  _m3508.motor_max_current = 8000; // 10000 max for c610+2006 16000 max for c620
  
  // output position = motor rotation(deg) * gear ratio.  2006 = 1:36 , 3508
  // = 1:19 rotate 180 deg
  //_m3508.set_position(0, 0);// set starting position as 0
  _m3508.set_velocity(0, 100);
  Timer pidTimer, printerTimer, espTimer;
  pidTimer.start();
  printerTimer.start();
  espTimer.start();
  int yaw = 0;
  Sender<int> sender(PA_9, PA_10);
    while(1){
       /* if(espTimer.elapsed_time() > 100ms){
            *sender.original = speed;
            sender.send();
            espTimer.reset();
        }*/
        if(pidTimer.elapsed_time() > 1ms){
            _m3508.c620_read();
            _m3508.c620_calc();
            _vesc1.set_rpm(can_id, speed);
            _vesc2.set_rpm(can_id2, speed);
            _m3508.set_velocity(0, -yaw);
            pidTimer.reset();
        }
        /*if (printerTimer.elapsed_time() > 1s){
            printf("speed: %d, fire: %d, mode: %c\r\n", speed, fire.read(), mm);
            //printf("RX_L: %d\n", RX_R);
            printerTimer.reset();
        }*/
        if(can.read(msg) && msg.id == 0x036){
             uint16_t LX_L = (uint16_t)(msg.data[0] << 8) | (msg.data[1]);
             uint16_t RX_R = (uint16_t)(msg.data[2] << 8) | (msg.data[3]);
             int8_t elva = (int8_t) msg.data[4];
             int8_t mode = (int8_t)msg.data[5];
            int8_t trig = (int8_t) msg.data[6];
            yaw = LX_L - 125;
            if (abs(yaw) < 15){
                yaw = 0;
            }
            printf("yaw: %d", yaw);
            //printf(" RX_R: %d  ", RX_R);
            //printf(" speed: %d ", speed);
            //printf(" trig: %c  \n", trig);
            //printf("pos: %d\t", yaw);
            //printf(" speed: %d \n", speed);

            if(elva == 'A'){
                speed += 2000;
            }
            if(elva == 'B'){
                speed -= 2000;
            }
            if(speed <= 0){
                    speed = 0; 
                }
            if(trig == '1'){
                if(mode =='S'){
                    fire = 1;
                }
                else{
                            fire = 1;
                            ThisThread::sleep_for(180);
                            fire = 0;
                            ThisThread::sleep_for(100);
                    
                }
               }
               else {
                fire = 0;
            }
            if(mode == 'S'){
                mm = 'S';
            }
            if(mode == 'A'){
                mm = 'A';
            }
        }
    }
}
