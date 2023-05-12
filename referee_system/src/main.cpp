#include "DJICRCHelper.hpp"
#include "NautilusSerialPort.hpp"
#include <spdlog/spdlog.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/uint8_t.h"

const int bufLength = 1025;
uint8_t buf[bufLength];
uint8_t cmdID[2];
DJICRCHelper djiCRCHelper;

FrameHeaderWithCmd frameHeadereWithCmd;
MatchStatus matchStatus;

int main()
{
    ros::init(argc, argv, "referee_system");
    ros::NodeHandle n;
    NautilusSerialPort sp;
    if (!sp.OpenPort("/dev/ttyUSB0", 115200, 0,8,1))
    {
        spdlog::error("Cannot Open Serial Port");
        return -1;
    }
    ros::Publisher heat_pub = n.advertise<std_msg::uint8_t>("sentry_heat", 50);
    ros::Publisher start_pub = n.advertise<std_msg::uint8_t>("judge_start", 3);
    ros::Publisher color_pub = n.advertise<std_msg::uint8_t>("judge_color", 10);

    while (ros::ok())
    {
		int i = 0;
		memset(buf, 0, sizeof(buf));
		int rawDataLength = 0;
		while (rawDataLength < bufLength - 1) {
			rawDataLength += sp.m_Receive(buf + rawDataLength, bufLength - 1 - rawDataLength);
		}
		while ( i < rawDataLength ) {
			if (buf[i] == 165) {
				// spdlog::info("Received header: {:X:n}", spdlog::to_hex(buf + i, buf + i + sizeof(FrameHeaderWithCmd)-2));
				if (djiCRCHelper.VerifyCRC8Checksum(buf + i, sizeof(FrameHeaderWithCmd) - 2))
				{
					// spdlog::info("Received header_cmd_id: {:X:n}", spdlog::to_hex(buf + i, buf + i + sizeof(FrameHeaderWithCmd)));
					size_t Data_length = ( buf[i + 2] << 8 ) | buf[i + 1];
					size_t lengthDataWithTail = Data_length + sizeof(FrameTail);
					// spdlog::info("cmd_id: {:X:n}", spdlog::to_hex(buf + i + 5, buf + i + 7));
					if (djiCRCHelper.VerifyCRC16Checksum(buf + i, sizeof(FrameHeaderWithCmd) + lengthDataWithTail)) {
						// spdlog::info("CRC16 correct");
						cmdID[0] = buf[i + 6]; cmdID[1] = buf[i + 5];
						if (cmdID[0] == 02 && cmdID[1] == 02) { // heat
							// spdlog::info("power_data: {:X:n}", spdlog::to_hex(buf + i + sizeof(FrameHeaderWithCmd), buf + i + sizeof(FrameHeaderWithCmd) + Data_length));
                            // spdlog::info("muzzle_heat: {:X:n}", spdlog::to_hex(buf + i + 10 + sizeof(FrameHeaderWithCmd), buf + i + sizeof(FrameHeaderWithCmd) + 14));
						}
						else if (cmdID[0] == 00 && cmdID[1] == 01) { // start
                            spdlog::info("game_status: {:X:n}", spdlog::to_hex(buf + i + sizeof(FrameHeaderWithCmd), buf + i + sizeof(FrameHeaderWithCmd) + 1));
                            spdlog::info("game_status_time: {:X:n}", spdlog::to_hex(buf + i + sizeof(FrameHeaderWithCmd) + 1, buf + i + sizeof(FrameHeaderWithCmd) + 3));
                            std::cout << "time: " << (int)buf[i + sizeof(FrameHeaderWithCmd) + 1] + (int)buf[i + sizeof(FrameHeaderWithCmd) + 2] * 256 << std::endl;
							// std::cout << "game starts la!" << std::endl;
						}
                        else if (cmdID[0] == 02 && cmdID[1] == 01) { // id
                            // spdlog::info("robot_id: {:X:n}", spdlog::to_hex(buf + i + sizeof(FrameHeaderWithCmd), buf + i + sizeof(FrameHeaderWithCmd) + 1));
                            // std::cout << "robot_id: " << (int)buf[i + sizeof(FrameHeaderWithCmd)] << std::endl;
                            if ((int)buf[i + sizeof(FrameHeaderWithCmd)] > 100) {
                                //send blue
                            } else {
                                //send red
                            }
                        }
					} else {
						// spdlog::info("CRC16 fail");
					}
					i += sizeof(FrameHeaderWithCmd) + lengthDataWithTail;
				}   
				else
				{
					// spdlog::info("CRC8 Failed");
					i++;
				}
			} else {
				i++;
			}
		} 
        ros::spinOnce();
    }

    return 0;
}

