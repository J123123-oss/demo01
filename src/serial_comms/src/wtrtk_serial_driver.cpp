#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <sstream>
#include "serial_comms/WTRTK.h" // 替换为你的功能包名

using namespace std;

// 解析$WTRTK字符串为数据结构
bool parseWTRTK(const string& data, serial_comms::WTRTK& msg) {
    if (data.substr(0, 6) != "$WTRTK") return false;

    // 去除帧头和校验位（*后的内容）
    size_t star_pos = data.find('*');
    if (star_pos == string::npos) return false;
    string content = data.substr(7, star_pos - 7); // 从$WTRTK,后开始提取

    // 按逗号分割字段
    vector<string> fields;
    stringstream ss(content);
    string field;
    while (getline(ss, field, ',')) {
        fields.push_back(field);
    }

    // 检查字段数量是否正确（共25个字段，不含帧头和校验位）
    if (fields.size() != 25) {
        ROS_WARN("Invalid WTRTK fields count: %zu", fields.size());
        return false;
    }

    // 填充消息（字段索引对应协议文档）
    try {
        msg.diff_x = stod(fields[0]);      // 差分X
        msg.diff_y = stod(fields[1]);      // 差分Y
        msg.diff_z = stod(fields[2]);      // 差分Z
        msg.diff_r = stod(fields[3]);      // 差分R
        msg.angle_x = stod(fields[4]);     // 角度X
        msg.angle_y = stod(fields[5]);     // 角度Y
        msg.angle_z = stod(fields[6]);     // 角度Z
        msg.fix_status = stoi(fields[7]);  // 定向状态
        msg.wireless_status = stoi(fields[8]); // 无线连接状态
        msg.ntrip_status = stoi(fields[9]);    // Ntrip状态
        msg.signal_quality = stoi(fields[10]); // 信号质量
        msg.data_rate = stoi(fields[11]);      // 数据量
        msg.gps_heading = fields[12];          // GPS航向角
        msg.calib_flag = stoi(fields[13]);     // 校准标志
        msg.battery_voltage = stod(fields[14]);// 电池电压
        msg.temperature = stod(fields[15]);    // 温度
        msg.base_distance = stoi(fields[16]);  // 基站距离
        msg.ins_flag = stoi(fields[17]);       // 惯导标志
        msg.ins_latitude = fields[18];         // 惯导纬度
        msg.lat_flag = fields[19];             // 纬度标志
        msg.ins_longitude = fields[20];        // 惯导经度
        msg.lon_flag = fields[21];             // 经度标志
        msg.ins_speed = stod(fields[22]);      // 惯导地速
        msg.ins_heading = stod(fields[23]);    // 惯导航向角
        msg.ins_altitude = stod(fields[24]);   // 惯导高度
    } catch (...) {
        ROS_WARN("Failed to parse WTRTK fields");
        return false;
    }
    return true;
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "wtrtk_serial_driver");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 声明参数（与nmea_navsat_driver类似）
    string port;
    int baud_rate;
    pnh.param<string>("port", port, "/dev/ttyUSB0"); // 默认端口
    pnh.param<int>("baud", baud_rate, 460800);      // 默认波特率

    // 初始化串口
    serial::Serial ser;
    try {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Failed to open serial port " << port << ": " << e.what());
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port " << port << " opened at " << baud_rate << " baud");
    } else {
        ROS_ERROR("Serial port not open");
        return -1;
    }

    // 创建发布者（话题名：/wtrtk_data）
    ros::Publisher wtrtk_pub = nh.advertise<serial_comms::WTRTK>("/wtrtk_data", 10);

    ros::Rate loop_rate(1); // 1Hz循环
    string buffer;

    while (ros::ok() && ser.isOpen()) {
        // 读取串口数据
        size_t n = ser.available();
        if (n > 0) {
            string data = ser.read(n);
            buffer += data;

            // 按换行符分割帧（NMEA协议以\r\n结尾）
            size_t pos;
            while ((pos = buffer.find("\r\n")) != string::npos) {
                string frame = buffer.substr(0, pos);
                buffer.erase(0, pos + 2);

                // 解析$WTRTK帧
                serial_comms::WTRTK msg;
                if (parseWTRTK(frame, msg)) {
                    msg.header.stamp = ros::Time::now(); // 设置时间戳
                    msg.header.frame_id = "wtrtk_link";  // 设置坐标系ID
                    wtrtk_pub.publish(msg);
                    ROS_DEBUG("Fix status: %d, Base distance: %d m", msg.fix_status, msg.base_distance);
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}