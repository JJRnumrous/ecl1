#include "ekf.h"
#include <vector>
#include <array>

#include <fstream>
#include <iostream>
#include <stdio.h>

std::vector<std::array<float,10>> imuData;
std::int64_t prev_stamp;

void read_imu(){

  std::ifstream inFile;    
  inFile.open("imu.dat", std::ios::in);    
  float  trash[5]; //tempf[10],
  std::array<float,10> tempf;

  while( !inFile.eof() ){
    inFile >> tempf[0] >> tempf[1] >> tempf[2] >> tempf[3] >> tempf[4] >> tempf[5] >> tempf[6] >> tempf[7] >> tempf[8] >> tempf[9] >> trash[0]>> trash[1]>> trash[2]>> trash[3]>> trash[4];
    imuData.push_back(tempf);
  }  
  inFile.close();
}

imuSample float2sample(std::array<float,10> imuD){
  imuSample output;
  output.time_us = imuD[0];
  output.delta_vel(0) = imuD[1];
  output.delta_vel(1) = imuD[2];
  output.delta_vel(2) = imuD[3];
  output.delta_ang(0) = imuD[4];
  output.delta_ang(1) = imuD[5];
  output.delta_ang(2) = imuD[6];
  output.delta_ang_dt = imuD[0] - prev_stamp;
  output.delta_vel_dt = imuD[0] - prev_stamp;
  prev_stamp = imuD[0];
  return output;
}




int main(){
    Ekf ekf2;    
    read_imu();

    for (auto& it : imuData)
    {      
      float pos[3], vel[3];
      Quatf quat;
      float temp[3]= {it[7], it[8], it[9]};
      uint64_t time = it[0];
      ekf2.init(time);
      ekf2.setIMUData(float2sample(it));
      
      ekf2.setMagData(it[0], temp );
      

      // ekf2.setGpsData(uint64_t time_usec, struct gps_message *gps);

      // ekf2.setExtVisionData(uint64_t time_usec, ext_vision_message *evdata);

      ekf2.update();

      quat = ekf2.calculate_quaternion();

      ekf2.get_position(pos);

      ekf2.get_velocity(vel);
      std::cout << pos << std::endl << vel << std::endl;
    }
}



