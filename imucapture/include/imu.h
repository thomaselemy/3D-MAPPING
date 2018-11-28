
#ifndef IMU_H
#define IMU_H

#include "deviceclass.h"

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>

#include <sys/types.h>
#include <string>
#include <pthread.h>
#include <mutex>

class IMU
{
public:
    inline int kbhit();
    void init(const std::string &port, int baud_rate);
    void device_initialization();
    void set_message_to_packet(
            XsDataPacket &packet,
            const XsMessage &msg,
            const XsDeviceId dev_id);
    void run();

    inline void
    print_position_to_file(std::ofstream &outfile, XsDataPacket &packet);
    inline void
    print_quaternion_to_file(std::ofstream &outfile, XsDataPacket &packet);
    inline void
    print_euler_to_file(std::ofstream &outfile, XsDataPacket &packet);
    inline void
    print_timestamp_to_file(std::ofstream &outfile, XsDataPacket &packet);
    
    inline void
    print_everything_to_console(XsDataPacket &packet);

private:
    std::mutex data_lock;

    DeviceClass device;
    XsPortInfo mtPort;
    IMU_data data;
};

struct IMU_data
{
	double latitude;
	double longitude;
	double altitude;
	double quaternion_w;
	double quaternion_x;
	double quaternion_y;
	double quaternion_z;
	double roll;
	double pitch;
	double yaw;
	int64_t time;
};

#endif // IMU_H