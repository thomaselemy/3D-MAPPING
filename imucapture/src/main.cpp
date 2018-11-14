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

inline int kbhit ()
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

auto scan_usb_devices()
{
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

void set_message_to_packet(
	XsDataPacket &packet,
	const XsMessage &msg,
	const XsDeviceId dev_id)
{
	auto msgID = msg.getMessageId();
	if (msgID == XMID_MtData) {
		LegacyDataPacket lpacket(1, false);
		lpacket.setMessage(msg);
		lpacket.setXbusSystem(false);
		lpacket.setDeviceId(dev_id, 0);
		lpacket.setDataFormat(
			XOM_Orientation,
			XOS_OrientationMode_Quaternion,
			0);
		XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
	} else if (msgID == XMID_MtData2) {
		packet.setMessage(msg);
		packet.setDeviceId(dev_id);
	}
}

inline void
print_position_to_file(std::ofstream &outfile, XsDataPacket &packet)
{
	XsVector position = packet.positionLLA();
	outfile << std::setw(15) << std::setprecision(5) << std::fixed
		<< position[0] << " "
		<< std::setw(15) << std::setprecision(5) << std::fixed
		<< position[1] << " "
		<< std::setw(15) << std::setprecision(5) << std::fixed
		<< position[2] << " ";
}

inline void
print_quaternion_to_file(std::ofstream &outfile, XsDataPacket &packet)
{
	XsQuaternion quaternion = packet.orientationQuaternion();
	outfile << std::setw(15) << std::setprecision(5) << std::fixed
		<< quaternion.w() << " "
		<< std::setw(15) << std::setprecision(5) << std::fixed
		<< quaternion.x() << " "
		<< std::setw(15) << std::setprecision(5) << std::fixed
		<< quaternion.y() << " "
		<< std::setw(15) << std::setprecision(5) << std::fixed
		<< quaternion.z() << " ";
}

inline void
print_euler_to_file(std::ofstream &outfile, XsDataPacket &packet)
{
	XsEuler euler = packet.orientationEuler();
	outfile << std::setw(15) << std::setprecision(5) << std::fixed
		<< euler.roll()  << " "
		<< std::setw(15) << std::setprecision(5) << std::fixed
		<< euler.pitch() << " "
		<< std::setw(15) << std::setprecision(5) << std::fixed
		<< euler.yaw()   << " ";
}

inline void
print_timestamp_to_file(std::ofstream &outfile, XsDataPacket &packet)
{
	outfile << std::setw(21) << std::setprecision(5) << std::fixed
		<< XsTime_timeStampNow(0);
}

inline void
print_everything(XsDataPacket &packet)
{
	XsVector position = packet.positionLLA();
	std::cout << "\r"
		<< "Lat:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< position[0]
		<< ",Lon:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< position[1]
		<< ",Alt:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< position[2]
		<< std::endl;
	
	XsVector3 velocity = packet.velocity();
	std::cout << "VelN:" << std::setw(15) << std::fixed << std::setprecision(5)
	<< velocity[0]
		<< ",VelE:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< velocity[1]
		<< ",VelD:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< velocity[2]
		<< std::endl;


	XsQuaternion quaternion = packet.orientationQuaternion();
	std::cout << "\r"
		<< "W:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< quaternion.w()
		<< ",X:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< quaternion.x()
		<< ",Y:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< quaternion.y()
		<< ",Z:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< quaternion.z();

	// Convert packet to euler
	XsEuler euler = packet.orientationEuler();
	std::cout << ",Roll:" << std::setw(15) << std::fixed << std::setprecision(5)
	<< euler.roll()
		<< ",Pitch:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< euler.pitch()
		<< ",Yaw:" << std::setw(15) << std::fixed << std::setprecision(5)
		<< euler.yaw()
		<< std::endl;

	// Get the Timestamp
	std::cout << ",msTime:" << std::setw(21) << std::fixed
	<< std::setprecision(5) << XsTime_timeStampNow(0);
}



int main(int argc, char* argv[])
{
	DeviceClass device;
	auto start = clock();
	auto mtPort = scan_usb_devices();

	try {
		device_initialization(device, mtPort);
	}
	catch (std::runtime_error const & error) {
		std::cout << error.what() << std::endl;
		exit(EXIT_FAILURE);
	}
	catch (...) {
		std::cout << "An unknown fatal error has occured. "
			<< "Aborting." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::cout << "Running..." << std::endl;

	XsByteArray data;
	XsMessageArray msgs;

	while (!kbhit()) {
		auto duration = (clock() - start) / CLOCKS_PER_SEC;

		device.readDataToBuffer(data);
		device.processBufferedData(data, msgs);
		for (auto msg : msgs) {
			
			XsDataPacket packet;
			set_message_to_packet(packet, msg, mtPort.deviceId());
			
			std::ofstream imu_txt(
				"IMU.txt",
				std::ofstream::out | std::ofstream::app);
			
			print_position_to_file(imu_txt, packet);
			print_quaternion_to_file(imu_txt, packet);
			print_euler_to_file(imu_txt, packet);
			print_timestamp_to_file(imu_txt, packet);
			print_everything(packet);

			imu_txt << std::endl;
			imu_txt.close();
		}
		msgs.clear();
		XsTime::msleep(0);
	}

	std::cout << "Closing port..." << std::endl;
	device.close();

	std::cout << "Successful exit." << std::endl;

	return 0;
}
