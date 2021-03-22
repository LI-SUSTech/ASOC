#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"


 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -45.0f
 #define V_MAX 45.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f



  ros::Publisher arrayPub;


double timeCounter;
struct can_frame rxframe;

int rxCounter = 0;
int trErrorCounter = 0;

std::mutex mutex;
int16_t angle, angle_last, angle_d, vel, R, I, temp, init;

void rxThread(int s) {
  init = 0;
  angle_last = 0;
  R=0;
  for (int i = 0;; i++) {
    int nbytes;
    struct can_frame rxframe;
    if (1) {
      nbytes = read(s, &rxframe, sizeof(struct can_frame));
      //  mutex.unlock();
      if (nbytes < 0) {
        perror("Read");
        break;
      }
      printf("0x%03X [%d] ", rxframe.can_id, rxframe.can_dlc);
      // for (i = 0; i < rxframe.can_dlc; i++)
      //   printf("%02X ", rxframe.data[i]);
      // printf("\r\n");

      angle = ((uint16_t)rxframe.data[0] << 8) + (uint16_t)rxframe.data[1];
      vel = ((uint16_t)rxframe.data[2] << 8) + (uint16_t)rxframe.data[3];
      I = ((uint16_t)rxframe.data[4] << 8) + (uint16_t)rxframe.data[5];
      temp = (int8_t)(uint16_t)rxframe.data[6];
      angle_d = angle - angle_last;

      int16_t motorID,pos,vel,I;
    motorID=rxframe.data[0];
    pos=((uint16_t)rxframe.data[1] << 8) + (uint16_t)rxframe.data[2];
    vel=((uint16_t)rxframe.data[3] << 8) + ((uint16_t)rxframe.data[4]<<4);


      if(angle_d<-4000){
        R++;
      }else if (angle_d>4000) {
      R--;
      }
      angle_last = angle;
    //   printf("motorID is %d ,pos is %d, Vel is %d, I is %d \r\n", angle, vel, I,
    //          temp);
      rxCounter++;
       printf("the %d  rx\r\n", rxCounter);
      int sizerx=sizeof(rxframe.data);
      for (int i ; i<sizerx; i++) {
      std::cout<<std::hex <<rxframe.data[i];
      }
      
     
    } else {
    }
    // printf("RX ");

    std::this_thread::sleep_for(
        std::chrono::nanoseconds(100000)); // 10 6600 9 7200   8  7700
  }
}



  int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }


void TmotorTest(struct can_frame &cmd){

//   int float_to_uint(float x, float x_min, float x_max, int bits){
//     /// Converts a float to an unsigned int, given range and number of bits ///
//     float span = x_max - x_min;
//     float offset = x_min;
//     return (int) ((x-offset)*((float)((1<<bits)-1))/span);
//     }
    
    
// float uint_to_float(int x_int, float x_min, float x_max, int bits){
//     /// converts unsigned int to float, given range and number of bits ///
//     float span = x_max - x_min;
//     float offset = x_min;
//     return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
//     }


float f_p, f_v,  f_kp,  f_kd,  f_t;

f_p=0;
f_v=0;

f_kp=6;
f_kd=1.2;
f_t=00;

f_p=10*sin(timeCounter/8);

uint16_t p, v, kp, kd, t;
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);


    cmd.data[0] = p>>8;
    cmd.data[1] = p&0xFF;
    cmd.data[2] = v>>4;
    cmd.data[3] = ((v&0xF)<<4)|(kp>>8);
    cmd.data[4] = kp&0xFF;
    cmd.data[5] = kd>>4;
    cmd.data[6] = ((kd&0xF)<<4)|(t>>8);
    cmd.data[7] = t&0xff;


}


void txThread(int s) {

  // for (int i = 0;; i++) {
  //   int nbytes;
  //   struct can_frame rxframe;
  //   nbytes = read(s, &rxframe, sizeof(struct can_frame));
  //   if (nbytes < 0) {
  //     perror("Read");
  //     break;
  //   }
  //   printf("0x%03X [%d] ", rxframe.can_id, rxframe.can_dlc);
  //   for (i = 0; i < rxframe.can_dlc; i++)
  //     printf("%02X ", rxframe.data[i]);
  //   printf("\r\n");
  //   rxCounter++;
  //   printf("the %d  rx\r\n", rxCounter);
  // }
  int targetPos = 4000;
  struct can_frame frame;
  for (int i = 0;; i++) { // i < 50000
    frame.can_id = 0x1;
    frame.can_dlc = 8;
    for (int j = 0; j < 8; j++) {
      frame.data[j] = 0x00;
    }
    // frame.data[0] = (i & 0xff000000) >> 24;
    // frame.data[1] = (i & 0x00ff0000) >> 16;
    // frame.data[2] = (i & 0x0000ff00) >> 8;
    // frame.data[3] = (i & 0x000000ff);
    // int16_t I = (500 - vel) * 30;

     int16_t I = (500 - (angle+R*8192)) * 0.3;
     I=0;
    frame.data[0] = I >> 8;
    frame.data[1] = I >> 0;

timeCounter+=0.01;
TmotorTest(frame);

std_msgs::Float64MultiArray sendArrary;

int test=sizeof(frame.data);

sendArrary.data.resize(test);
for (int i=0; i<test; i++) {
sendArrary.data[i]=frame.data[i];
}
arrayPub.publish(sendArrary);


    int nbytes;
    if (1) {
      //  mutex.lock();
      nbytes = write(s, &frame, sizeof(struct can_frame));
      // mutex.unlock();
      // printf("Wrote %d bytes\n", nbytes);
      if (nbytes == -1) {
        printf("send error\n");
        trErrorCounter++;
        //   return 0;
      }
    }

    std::this_thread::sleep_for(
        std::chrono::nanoseconds(1000000)); // 10 6600 9 7200   8  7700
  }

  printf("tx over \n");
}

int main(int argc, char** argv) {


  ros::init(argc, argv, "tmotorTest");
  ros::NodeHandle node;
arrayPub=node.advertise<std_msgs::Float64MultiArray>("tmotorArray",1);
    //   std_msgs::Float64MultiArray testmsg;
    //   testmsg.data.resize(8);
    //   arrayPub.publish(testmsg);
  int s;
  int nbytes;
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  angle = 0;
  const char *ifname = "can0";

  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket");
    return -1;
  }

  strcpy(ifr.ifr_name, ifname);
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Error in socket bind");
    return -2;
  }

/// star motor
   frame.can_dlc = 8;
frame.can_id=0x001;
for (int i =0; i<8; i++) {
frame.data[i]=0xff;
}
frame.data[7]=0xfc;
   nbytes = write(s, &frame, sizeof(struct can_frame));
      // mutex.unlock();
      // printf("Wrote %d bytes\n", nbytes);
      if (nbytes == -1) {
        printf("send error\n");
      }


  std::thread canTx(txThread, s);
  std::thread canRx(rxThread, s);

  canRx.join();
  // canTx.join();

  printf("send over\n");
  printf(" trErrorCounter is  %d  \r\n", trErrorCounter);
  printf("the final number rx is  %d  \r\n", rxCounter);
  return 0;
}
