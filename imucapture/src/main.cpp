/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "deviceclass.h"
#include "conio.h"

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xsens/xstimestamp.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <fstream>
#include <ctime>

#include <cstdio>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

void changemode(int dir);
int kbhit();
void device_initialization(DeviceClass &device, XsPortInfo &mtPort);
XsPortInfo scan_usb_devices();

int main(int argc, char* argv[])
{
	DeviceClass device;
	auto start = clock();
	XsPortInfo mtPort;
	mtPort = scan_usb_devices();

	try {
		device_initialization(device, mtPort);
	}
	catch (std::runtime_error const & error) {
		std::cout << error.what() << std::endl;
		exit(EXIT_FAILURE);
	}
	catch (...) {
		std::cout << "An unknown fatal error has occured. " <<
				     "Aborting." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::cout << std::endl << "Main loop (press any key to quit)" << std::endl;
	std::cout << std::string(79, '-') << std::endl;

	XsByteArray data;
	XsMessageArray msgs;

	while (!kbhit()) {
		//TODO:Move to chrono
		auto duration = (clock() - start) / CLOCKS_PER_SEC;

		device.readDataToBuffer(data);
		device.processBufferedData(data, msgs);
		for (auto msg : msgs) {
			// Retrieve a packet
			XsDataPacket packet;
			auto msgID = msg.getMessageId();
			if (msgID == XMID_MtData) {
				LegacyDataPacket lpacket(1, false);
				lpacket.setMessage(msg);
				lpacket.setXbusSystem(false);
				lpacket.setDeviceId(mtPort.deviceId(), 0);
				lpacket.setDataFormat(
						XOM_Orientation,
						XOS_OrientationMode_Quaternion,
						0);
				XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
			
			}else if (msgID == XMID_MtData2) {
				packet.setMessage(msg);
				packet.setDeviceId(mtPort.deviceId());
			}

			XsVector position = packet.positionLLA();
			std::cout << "\r"
				<< "Lat:" << std::setw(5) << std::fixed << std::setprecision(std::numeric_limits<long double>::digits10) << position[0]
				<< ",Lon:" << std::setw(5) << std::fixed << std::setprecision(std::numeric_limits<long double>::digits10) << position[1]
				<< ",Alt:" << std::setw(5) << std::fixed << std::setprecision(3) << position[2]
				;
			XsVector3 velocity = packet.velocity();
			std::cout << "VelN:" << std::setw(7) << std::fixed << std::setprecision(3) << velocity[0]
				<< ",VelE:" << std::setw(7) << std::fixed << std::setprecision(3) << velocity[1]
				<< ",VelD:" << std::setw(7) << std::fixed << std::setprecision(3) << velocity[2]
				;

			// Get the quaternion data
			XsQuaternion quaternion = packet.orientationQuaternion();
			std::cout << "\r"
				<< "W:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.w()
				<< ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.x()
				<< ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.y()
				<< ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.z()
				;

			// Convert packet to euler
			XsEuler euler = packet.orientationEuler();
			std::cout << ",Roll:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.roll()
				<< ",Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.pitch()
				<< ",Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.yaw()
				;

			// Get the Timestamp
			std::cout << ",msTime:" << std::setw(12) << std::fixed << std::setprecision(2) << XsTime_timeStampNow(0);

			/*// fstreaming to a text file
			{
				std::ofstream textfileoutput;
				textfileoutput.open("IMU.txt", std::ofstream::out | std::ofstream::app);
				// Titles (first line)
				//textfileoutput << "w,x,y,z,Roll,Pitch,Yaw,Time" << std::endl;

				// Titles (second line)
				//textfileoutput << "Lat,Lon,Alt,Vel_N,Vel_E,Vel_D,w,x,y,z,Roll,Pitch,Yaw,Time" << std::endl;

				// lat/lon/alt
				auto position = packet.positionLLA();
				textfileoutput << std::setw(15) << std::fixed << std::setprecision(6) << std::showpoint << std::right << position[0]
					<< std::setw(15) << std::fixed << std::setprecision(6) << std::showpoint << std::right << position[1]
					<< std::setw(15) << std::fixed << std::setprecision(6) << std::showpoint << std::right << position[2]
					;

				// Get the quaternion data
				auto quaternion = packet.orientationQuaternion();
				textfileoutput << "\r"
					<< "" << std::setw(15) << std::right << std::setprecision(5) << std::showpoint << quaternion.w()
					<< "" << std::setw(15) << std::right << std::setprecision(5) << std::showpoint << quaternion.x()
					<< "" << std::setw(15) << std::right << std::setprecision(5) << std::showpoint << quaternion.y()
					<< "" << std::setw(15) << std::right << std::setprecision(5) << std::showpoint << quaternion.z()
					;

				// Convert packet to euler
				auto euler = packet.orientationEuler();
				textfileoutput << "" << std::setw(15) << std::fixed << std::showpoint << std::right << std::setprecision(5) << euler.roll()
					<< "" << std::setw(15) << std::fixed << std::showpoint << std::right << std::setprecision(5) << euler.pitch()
					<< "" << std::setw(15) << std::fixed << std::showpoint << std::right << std::setprecision(5) << euler.yaw()
					;

				// Get the Timestamp
				textfileoutput << " " << std::setw(20) << std::fixed << std::showpoint << std::right << XsTime_timeStampNow(0) << "\n"
					;


				//velocity
				//XsVector3 velocity = packet.velocity();
				//textfileoutput << std::setw(15) << std::fixed << std::setprecision(3) << std::showpoint << std::right << velocity[0]
				//	<< std::setw(15) << std::fixed << std::setprecision(3) << std::showpoint << std::right << velocity[1]
				//	<< std::setw(15) << std::fixed << std::setprecision(3) << std::showpoint << std::right << velocity[2]
				//	;


				textfileoutput.close();

			}*/

			std::cout << std::flush;
		}
		msgs.clear();
		XsTime::msleep(0);
	}
	std::cin.get();
	std::cout << std::endl << std::string(79, '-') << std::endl;

	std::cout << "Closing port..." << std::endl;
	device.close();

	std::cout << "Successful exit." << std::endl;
	std::cout << "Press [ENTER] to continue." << std::endl;
	std::cin.get();

	return 0;
}



void changemode(int dir)
{
    static struct termios oldt, newt;

    if (dir == 1) {
        tcgetattr( STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    } else {
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
	}
}

int kbhit ()
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);

    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
}

void device_initialization(DeviceClass &device, XsPortInfo &mtPort)
{
	std::cout << "Opening port..." << std::endl;
	if (!device.openPort(mtPort)) {
		throw std::runtime_error("Could not open port. Aborting.");
	}
	
	std::cout << "Putting device into configuration mode..." << std::endl;
	if (!device.gotoConfig()) {
		throw std::runtime_error("Could not put device \
								  into configuration mode. Aborting.");
	}

	// Request the device Id to check the device type
	mtPort.setDeviceId(device.getDeviceId());

	// Check if we have an MTi / MTx / MTmk4 device
	if (!mtPort.deviceId().isMt9c() &&
		!mtPort.deviceId().isLegacyMtig() &&
		!mtPort.deviceId().isMtMk4() &&
		!mtPort.deviceId().isFmt_X000())
	{
		throw std::runtime_error("No MTi / MTx / MTmk4 device found. \
								  Aborting.");
	}

	std::cout << "Found a device with id: "
			  << mtPort.deviceId().toString().toStdString()
			  << " @ port: "	 << mtPort.portName().toStdString()
			  << ", baudrate: "  << mtPort.baudrate() << std::endl;

	// Print information about detected MTi / MTx / MTmk4 device
	std::cout << "Device: " << device.getProductCode().toStdString()
			  << " opened." << std::endl;

	// Note the differences between MTix and MTmk4
	std::cout << "Configuring the device..." << std::endl;
	if (mtPort.deviceId().isMt9c() || mtPort.deviceId().isLegacyMtig())
	{
		// output orientation data
		XsOutputMode outputMode = XOM_Orientation;

		// output orientation data as quaternion
		XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion;

		// set the device configuration
		if (!device.setDeviceMode(outputMode, outputSettings))
			throw std::runtime_error("Could not configure MT device. \
									  Aborting.");
			
	}
	else if (mtPort.deviceId().isMtMk4() || mtPort.deviceId().isFmt_X000())
	{
		XsOutputConfiguration quat(XDI_Quaternion, 100);
		XsOutputConfigurationArray configArray;
		configArray.push_back(
			XsOutputConfiguration(XDI_LatLon | XDI_SubFormatDouble, 100)
		);
		configArray.push_back(
			XsOutputConfiguration(XDI_AltitudeEllipsoid, 100)
		);
		configArray.push_back(
			XsOutputConfiguration(XDI_Quaternion | XDI_CoordSysNed, 100)
		);
		configArray.push_back(
			XsOutputConfiguration(XDI_VelocityXYZ | XDI_CoordSysNed, 100)
		);

		configArray.push_back(quat);

		if (!device.setOutputConfiguration(configArray))
			throw std::runtime_error("Could not configure MTmk4 device. \
									  Aborting.");
	}
	else
	{
		throw std::runtime_error("Unknown device while configuring. Aborting.");
	}

	std::cout << "Putting device into measurement mode..." << std::endl;
	if (!device.gotoMeasurement()) {
		throw std::runtime_error("Could not put device into measurement mode. \
								  Aborting.");
	}
}

XsPortInfo scan_usb_devices() {
	std::cout << "Scanning for USB devices..." << std::endl;
	XsPortInfoArray portInfoArray;
	xsEnumerateUsbDevices(portInfoArray);
	if (!portInfoArray.size()) // Can't find device
	{
		std::string portName;
		int baudRate;
		std::cout << "No USB Motion Tracker found." << std::endl << std::endl
				  << "Enter COM port name (eg. /dev/ttyUSB0): " << std::endl;
		std::cin >> portName;
		std::cout << "Enter baud rate (eg. 115200): ";
		std::cin >> baudRate;

		XsPortInfo portInfo(portName, XsBaud::numericToRate(baudRate));
		portInfoArray.push_back(portInfo);
	}

	// Use the first detected device
	return portInfoArray.at(0);
}
