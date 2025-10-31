#include <ros/ros.h>
#include <modbus/modbus.h>
#include <serial_comms/Environment.h>  // 自定义消息头文件
#include <stdint.h>

// 寄存器地址定义（十进制）
#define REG_WIND_SPEED     500   // 风速（实际值的10倍）
#define REG_WIND_DIR       503   // 风向（实际值）
#define REG_LUX_HIGH       510   // 光照高16位
#define REG_LUX_LOW        511   // 光照低16位
#define REG_RAINFALL       513   // 雨量（实际值的10倍）

// Modbus RTU配置参数（根据实际硬件修改）
#define MODBUS_DEVICE      "/dev/ttyUSB0"  // 串口设备
#define BAUDRATE           4800            // 波特率
#define PARITY             'N'             // 校验位（N无/E偶/O奇）
#define DATA_BITS          8               // 数据位
#define STOP_BITS          1               // 停止位
#define SLAVE_ID           1               // 从机地址

int main(int argc, char**argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "weather_station_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<serial_comms::Environment>(
        "/environment_data", 10);  // 发布话题

    // 初始化Modbus上下文
    modbus_t *ctx = modbus_new_rtu(MODBUS_DEVICE, BAUDRATE, PARITY, DATA_BITS, STOP_BITS);
    if (ctx == NULL) {
        ROS_FATAL("无法初始化Modbus RTU上下文: %s", modbus_strerror(errno));
        return -1;
    }

    // 设置从机地址
    modbus_set_slave(ctx, SLAVE_ID);
    if (modbus_connect(ctx) == -1) {
        ROS_FATAL("无法连接到Modbus设备: %s", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    ros::Rate loop_rate(0.5);  // 0.5Hz发布频率
    uint16_t reg_buffer[10];  // 寄存器数据缓冲区

    while (ros::ok()) {
        serial_comms::Environment msg;

        // 1. 读取风速（寄存器500，1个寄存器）
        if (modbus_read_registers(ctx, REG_WIND_SPEED, 1, reg_buffer) == 1) {
            msg.wind_speed = reg_buffer[0] / 10.0f;  // 实际值 = 寄存器值 / 10
        } else {
            ROS_WARN("读取风速寄存器失败: %s", modbus_strerror(errno));
        }

        // 2. 读取风向（寄存器503，1个寄存器）
        if (modbus_read_registers(ctx, REG_WIND_DIR, 1, reg_buffer) == 1) {
            msg.wind_direction = reg_buffer[0];  // 直接为实际角度
        } else {
            ROS_WARN("读取风向寄存器失败: %s", modbus_strerror(errno));
        }

        // 3. 读取光照强度（高16位+低16位，连续2个寄存器）
        if (modbus_read_registers(ctx, REG_LUX_HIGH, 2, reg_buffer) == 2) {
            uint32_t lux_high = reg_buffer[0];  // 高16位
            uint32_t lux_low = reg_buffer[1];   // 低16位
            msg.illuminance = (lux_high << 16) | lux_low;  // 拼接32位值
        } else {
            ROS_WARN("读取光照寄存器失败: %s", modbus_strerror(errno));
        }

        // 4. 读取雨量（寄存器513，1个寄存器）
        if (modbus_read_registers(ctx, REG_RAINFALL, 1, reg_buffer) == 1) {
            msg.rainfall = reg_buffer[0] / 10.0f;  // 实际值 = 寄存器值 / 10
        } else {
            ROS_WARN("读取雨量寄存器失败: %s", modbus_strerror(errno));
        }

        msg.stamp = ros::Time::now();  // 时间戳
        pub.publish(msg);              // 发布消息

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 关闭连接并释放资源
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}