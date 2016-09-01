#include <memory>
#include "ros/ros.h"
#include <ros/package.h>
#include <boost/asio.hpp>
#include "std_msgs/UInt8.h"
#include <boost/format.hpp>

class SerialDevice {
public:
    SerialDevice(std::string port, unsigned baudrate);
    void read(uint8_t* c, unsigned size=1);

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial_port;
};

SerialDevice::SerialDevice(std::string port, unsigned baudrate)
    : serial_port(io, port)
{
    serial_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
}

void SerialDevice::read(uint8_t *c, unsigned size)
{
    boost::asio::read(serial_port, boost::asio::buffer(c, size));
}

int main(int argc, char* argv[]){

    std::unique_ptr<SerialDevice> SD;
    bool doaDeviceFound = false;
    uint8_t _doa_raw;

    ros::init(argc, argv, "acoustic_magic_doa");
    ros::NodeHandle nh("~");

    std::string doa_serial_device;
    std::string topic_name;
    nh.param("acoustic_magic_doa/topic_name", topic_name, std::string("/acoustic_magic_doa/data_raw"));
    nh.param("acoustic_magic_doa/serial_port", doa_serial_device, std::string("/dev/ttyUSB0"));
    ros::Publisher pub = nh.advertise<std_msgs::UInt8>(topic_name, 5);
    std_msgs::UInt8 msg;


    try{
        SD.reset(new SerialDevice(doa_serial_device, 2400)); 
        doaDeviceFound = true;
    }
    catch(std::exception & e){
        ROS_ERROR("MIC ARRAY DEVICE NOT FOUND");
    }

    while(ros::ok()){
        if(doaDeviceFound){
            SD->read(&_doa_raw);
            msg.data = _doa_raw;
            pub.publish(msg);
            //printf("%d\n", _doa_raw);
        }
    }
}