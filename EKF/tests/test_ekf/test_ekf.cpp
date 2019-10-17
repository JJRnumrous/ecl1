/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file base.cpp
 *
 * Tests for the estimator base class
 */

#include <EKF/ekf.h>

#include <vector>
#include <array>
#include <string>
#include <sstream>
#include <fstream>
#include <stdio.h>

#include <cstdio>
#include <random>
#include <iostream>
#include <mathlib/mathlib.h>

std::vector<std::array<float,11>> imuData;
std::vector<gps_message> gpsData;
std::vector<ext_vision_message> visData;
std::int64_t prev_stamp = 0;
imuSample prev_IMU;

// Read in the IMU data from file
//File saved in format:
//time_us accelX accelY accelZ GyroX GyroY GyroZ MagX MagY MagZ [vector(1,5)containing Baro measurements]
void read_imu(){

  std::ifstream inFile;    
  inFile.open("imu.dat", std::ios::in);    
  float  trash[4]; //tempf[10],
  std::array<float,11> tempf;

  while( !inFile.eof() ){
    inFile >> tempf[0] >> tempf[1] >> tempf[2] >> tempf[3] >> tempf[4] >> tempf[5] >> tempf[6] >> tempf[7] >> tempf[8] >> tempf[9] >> trash[0]>> trash[1]>> tempf[10]>> trash[2]>> trash[3];
	// std::cout << " " << tempf[0] << " " << tempf[1] << " " << tempf[2] << " " << tempf[3] << " " << tempf[4] << " " << tempf[5] << " " << tempf[6] << " " << tempf[7] << " " << tempf[8] << " " << tempf[9] << " " << trash[0]<< " " << trash[1]<< " " << tempf[10]<< " " << trash[2]<< " " << trash[3] << std::endl;	
    imuData.push_back(tempf);
  }  
  inFile.close();
}

void read_gps(){
	std::ifstream inFile;    
  	inFile.open("gps.dat", std::ios::in);  
  	gps_message gps;
	std::string line;
  	int value;
  	int nsats;
  	while( std::getline(inFile, line) ) {    
		std::istringstream iss(line);	
    	iss >> gps.time_usec >> gps.lat >> gps.lon >> gps.alt >> gps.eph >> gps.epv >> gps.vel_m_s >> gps.vel_ned[0] >> gps.vel_ned[1] >> gps.vel_ned[2] >> value >> gps.fix_type >> nsats;		
		gps.lat = gps.lat;
		gps.lon = gps.lon;
		gps.nsats = nsats;
		gps.alt = gps.alt;
		gps.vel_m_s = gps.vel_m_s;
		gps.vel_ned[0] = gps.vel_ned[0];
		gps.vel_ned[1] = gps.vel_ned[1];
		gps.vel_ned[2] = gps.vel_ned[2];
		gps.eph = 1.0f;
		gps.epv = 1.0f;
		gps.yaw = math::radians(value/100.0f -180.0f);
		gps.yaw_offset = 0.0f;
		gps.vel_ned_valid = false;
		gps.sacc = 5.0f;
		gps.gdop = 0.0f; //not sure
		gpsData.push_back(gps);
  	}  
  	inFile.close();
}

void read_vis(){
  
  std::ifstream inFile;    
  inFile.open("vision.dat", std::ios::in);  
  ext_vision_message vis;  
  float temp;  
  while( !inFile.eof() ){    
    inFile >> temp >> vis.posNED(1) >> vis.posNED(0) >> vis.posNED(2) >> vis.quat(0) >> vis.quat(1) >> vis.quat(2) >> vis.quat(3);	
	// std::cout << " " << temp << " " << vis.posNED(0) << " " << vis.posNED(1) << " " << vis.posNED(2) << " " << vis.quat(0) << " " << vis.quat(1) << " " << vis.quat(2) << " " << vis.quat(3) << std::endl;	

    // TO test the EKF	
	// vis.posNED(0) = vis.posNED(1);
	// vis.posNED(1) = vis.posNED(0);
	vis.posNED(2) = -vis.posNED(2);
	// vis.quat(0) = vis.quat(0);
	vis.quat(1) = -vis.quat(1);
	// vis.quat(2) = vis.quat(2);
	vis.quat(3) = -vis.quat(3);
	vis.posErr = temp;
	vis.hgtErr = 0.01f;
	vis.angErr = 0.01f;	
	visData.push_back(vis);
  }  
  inFile.close();
}

imuSample float2sample(std::array<float,11> imuD){
  imuSample output;
  output.time_us = imuD[0];
  output.delta_ang_dt = (imuD[0] - prev_IMU.time_us)*1e-6f; // from us to sec
  output.delta_vel_dt = (imuD[0] - prev_IMU.time_us)*1e-6f; // from us to sec
  
  // according to the EKF2 main wrapper the delta_vel and delta_ang is the accelerometer*dt and gyro*dt   repsectively

	//   output.delta_vel(0) = (imuD[1]- prev_IMU.delta_vel(0))*output.delta_vel_dt;
	//   output.delta_vel(1) = (imuD[2]- prev_IMU.delta_vel(1))*output.delta_vel_dt;
	//   output.delta_vel(2) = (imuD[3]- prev_IMU.delta_vel(2))*output.delta_vel_dt; 
	//   output.delta_ang(0) = (imuD[4]- prev_IMU.delta_ang(0))*output.delta_ang_dt;
	//   output.delta_ang(1) = (imuD[5]- prev_IMU.delta_ang(1))*output.delta_ang_dt;
	//   output.delta_ang(2) = (imuD[6]- prev_IMU.delta_ang(2))*output.delta_ang_dt;
	
  output.delta_vel(0) = (imuD[1])*output.delta_vel_dt;
  output.delta_vel(1) = (imuD[2])*output.delta_vel_dt;
  output.delta_vel(2) = (imuD[3])*output.delta_vel_dt;
  output.delta_ang(0) = (imuD[4])*output.delta_ang_dt;
  output.delta_ang(1) = (imuD[5])*output.delta_ang_dt;
  output.delta_ang(2) = (imuD[6])*output.delta_ang_dt;
  
  prev_IMU.time_us = imuD[0];
  prev_IMU.delta_vel(0) = imuD[1];
  prev_IMU.delta_vel(1) = imuD[2];
  prev_IMU.delta_vel(2) = imuD[3];  
  prev_IMU.delta_ang(0) = imuD[4];
  prev_IMU.delta_ang(1) = imuD[5];
  prev_IMU.delta_ang(2) = imuD[6];
  
  return output;
}

void clear_output(bool gps_en){
	std::ofstream outFile;   
	if(gps_en){         
		outFile.open("out_gps.dat", std::ios::trunc);    
	}else{
		outFile.open("out_vis.dat", std::ios::trunc);    
	}
    outFile.close();
}

void write_imu(float* pos, float* vel, Quatf quat, uint64_t stamp, bool gps_en, uint32_t status, uint16_t fault){
  std::ofstream outFile;            
  if(gps_en){
	outFile.open("out_gps.dat", std::ios::app);  
  }else{
	outFile.open("out_vis.dat", std::ios::app);  
  }
  outFile << 1.0f*stamp*1e-6f << "\t\t[ " << pos[0] << ", " << pos[1] << ", " << pos[2] << " ]\t\t[ " <<  quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3) << " ]\t\t[ " << vel[0] << ", " << vel[1] << ", " << vel[2] << " ] [ " << status << " ] [ " << fault << " ]" << std::endl;  
  outFile.close();
}



int main(int argc, char *argv[])
{
	bool gps_enabled = false;
	(void)argc; // unused
	(void)argv; // unused

	Ekf *ekf = new Ekf();	
	std::cout << "EKF created" << std::endl;
	read_imu();
	std::cout << "IMU data read" << std::endl;

	if(gps_enabled){
		read_gps();
		std::cout << "GPS data read" << std::endl;
	}else{
		read_vis();
		std::cout << "Vision data read" << std::endl;
	}
	clear_output(gps_enabled);	
	std::ofstream myFile;
	myFile.open("EKF.dat");
	ext_vision_message evData = {};
	ekf->init(0);		
	for (auto& it : imuData)
	{   
		float pos[3], vel[3];
		Quatf quat;
		Eulerf euler_out(quat);
		

		float temp[3]= {it[7], it[8], it[9]};
		uint64_t time = it[0];	  
		imuSample tempImu = float2sample(it); 
		// std::cout << temp.time_us << " [ " << temp.delta_vel(0) << " " << temp.delta_vel(1) << " " << temp.delta_vel(2) << " ],[ " << temp.delta_ang(0) << " " << temp.delta_ang(1)  << " " << temp.delta_ang(2) << " ],[ " << temp.delta_ang_dt << " " << temp.delta_vel_dt << " ]" << std::endl;
		// std::cout << time << " " << temp[0] << ", " << temp[1] << ", " << temp[2] << " " << it[10] << std::endl;
		if(time%8000 ==0){
			ekf->setIMUData(tempImu); 	
			ekf->setMagData(time, temp );	 	
			ekf->setBaroData(time, it[10] );
		}
		


		if(gps_enabled){
			// for (auto& ti : gpsData){		  
				// if(abs(ti.time_usec - time) < 2000){					
				// 	ekf->setGpsData(time, ti);				
				// 	// ekf->update();
				// }
			// }	
		}else{		  
			for (auto&ti : visData) {
				if(fabs(ti.posErr*1e6f - time)<2000.0f){								
					ti.posErr = 0.01f;				
					ekf->setExtVisionData(time, &ti);
					evData.quat = ti.quat;
					evData.posNED = ti.posNED;							  			
				}
			}		
		}	  
		// std::cout << time << " " << ekf->update() << std::endl;	
		ekf->update();;
		quat = ekf->calculate_quaternion();
		ekf->get_position(pos);
		ekf->get_velocity(vel);
		euler_out = Eulerf(quat);
		Eulerf euler_init(evData.quat);
		uint32_t status;
		uint16_t fault;
		ekf->get_control_mode(&status);
		ekf->get_filter_fault_status(&fault);
		write_imu(pos, vel, quat, time, gps_enabled, status, fault);	

		// std::cout << quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3) << std::endl;
		myFile << time << ", " << quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3) << ", "<< vel[0] << ", " << vel[1] << ", " << vel[2] << ", " << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " <<  euler_out(0)*180.0f/M_PI << ", " <<  euler_out(1)*180.0f/M_PI << ", " <<  euler_out(2)*180.0f/M_PI << ", " <<  evData.quat(0) << ", " <<  evData.quat(1) << ", " <<  evData.quat(2) << ", " <<  evData.quat(3) << ", " <<  evData.posNED(0) << ", " <<  evData.posNED(1) << ", " <<  evData.posNED(2) << ", "  << euler_init(0)*180.0f/M_PI << ", "  << euler_init(1)*180.0f/M_PI << ", "  << euler_init(2)*180.0f/M_PI << std::endl;
		
	}
	myFile.close();
	delete ekf;

	return 0;
}

// int main(int argc, char *argv[])
// {
// 	(void)argc; // unused
// 	(void)argv; // unused

// 	std::ofstream myFile;
//     myFile.open("EKF.dat");

// 	Ekf *ekf = new Ekf();
//     uint64_t time_usec = 0;		// simulation start time
// 	int sample_period  = 4000;

// 	ekf->init(time_usec);

// 	imuSample imu = {};
//     imu.delta_vel    = { 0.0f, 0.0f, -CONSTANTS_ONE_G*sample_period*1e-6f};
//     //imu.delta_vel    = { 0.0f, 0.0f, 0.0f};
//     imu.delta_ang    = { 0.0f, 0.0f, 0.0f};
//     imu.delta_ang_dt = sample_period*1e-6f;
//     imu.delta_vel_dt = sample_period*1e-6f;
//     imu.time_us      = time_usec;

//     ext_vision_message evData = {};
//     evData.posNED(0) = 10.0f;
//     evData.posNED(1) = 20.0f;
//     evData.posNED(2) = 30.0f;

  
//     Eulerf euler_init(5.0f/180.0f*M_PI, -10.0f/180.0f*M_PI, 45.0f/180.0f*M_PI);
// 	// // Eulerf euler_init(0.0f*M_PI/180.0f, 0.0f*M_PI/180.0f, 0.0f*M_PI/180.0f);
	
//     evData.quat = Quatf(euler_init);
    

//     evData.posErr = 0.001;
//     evData.hgtErr = 0.001;
//     evData.angErr = 0.001;

//     float pos[3];
//     float vel[3];
//     Quatf quat;
//     Eulerf euler_out(quat);

// 	// simulate two seconds
// 	for (int i = 1; i < 2000; i++) {
// 	    if (i == 100) {
//             evData.posNED(0) = 10.0f;
// 			// euler_init = Eulerf(0.0f/180.0f*M_PI, 0.0f/180.0f*M_PI, 0.0f/180.0f*M_PI);
// 			// evData.quat = Quatf(euler_init);
// 	    }
//         time_usec += sample_period;   	

//         imu.time_us = time_usec;
//         ekf->setIMUData(imu);

// 	//        if (i % 10 == 0) {
// 	//            gps.time_usec = time_usec;
// 	//            ekf->setGpsData(time_usec,gps);
// 	//        }

//         if (i % 8 == 0) {
//             ekf->setExtVisionData(time_usec, &evData);
//         }

//         ekf->update();
//         quat = ekf->calculate_quaternion();
//         ekf->get_velocity(vel);
//         ekf->get_position(pos);
//         euler_out = Eulerf(quat);
// 		euler_out(0) = euler_out(0)*180.0f/M_PI;
// 		euler_out(1) = euler_out(1)*180.0f/M_PI;
// 		euler_out(2) = euler_out(2)*180.0f/M_PI;


		
		
//         // std::cout << time_usec*1e-6 << ", " << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " <<  quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3) << ", " << vel[0] << ", " << vel[1] << ", " << vel[2] << std::endl;
// 		std::cout << time_usec*1e-6 << ",[ " << pos[0] << ", " << pos[1] << ", " << pos[2] <<  " ],[ " <<  euler_out(0) << ", " <<  euler_out(1) << ", " <<  euler_out(2)<< " ],[ " <<  quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3) <<std::endl;
//         myFile << time_usec*1e-6 << ", " << quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3) << ", "<< vel[0] << ", " << vel[1] << ", " << vel[2] << ", " << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " <<  euler_out(0) << ", " <<  euler_out(1) << ", " <<  euler_out(2) << ", " <<  evData.quat(0) << ", " <<  evData.quat(1) << ", " <<  evData.quat(2) << ", " <<  evData.quat(3) << ", " <<  evData.posNED(0) << ", " <<  evData.posNED(1) << ", " <<  evData.posNED(2) << ", "  << euler_init(0) << ", "  << euler_init(1) << ", "  << euler_init(2) << std::endl;
// 	}
// 	myFile.close();
//     //std::cout << "truequat = " << evData.quat(0) << ", " << evData.quat(1) << ", " << evData.quat(2) << ", " <<  evData.quat(3) << std::endl;
//     //Eulerf euler_out(quat);
//     //std::cout << "euler_out = " << euler_out(0)*180.0f/M_PI << ", " << euler_out(1)*180.0f/M_PI << ", " << euler_out(3)*180.0f/M_PI << std::endl;

// 	delete ekf;

// 	return 0;
// }