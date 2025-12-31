#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import subprocess
from std_msgs.msg import Bool, UInt8
from serial_comms.msg import Sensors


class ProximitySensorReader:
    def __init__(self):
        rospy.init_node('proximity_input_reader')
        
        # 保留原有ROS参数（兼容原配置，无用参数仅保留不影响逻辑）
        self.device_address = rospy.get_param('~device_address', 1)
        self.can_interface = rospy.get_param('~can_interface', 'can0')
        self.update_rate = rospy.get_param('~update_rate', 20.0)  # 轮询频率(Hz)
        self.baudrate = rospy.get_param('~baudrate', 1000000)
        self.request_baudrate = rospy.get_param('~request_baudrate', False)

        # ========== GPIO配置（核心）==========
        # 定义两个接近开关对应的GPIO引脚（wiringOP编号，根据实际接线修改）
        self.proximity_pins = {
            'sensor_a': 5,   # 第一个接近开关GPIO（示例：5号引脚）
            'sensor_b': 6    # 第二个接近开关GPIO（示例：6号引脚）
            # 如需扩展可继续添加 sensor_c/sensor_d
        }
        
        # GPIO初始化标志
        self.gpio_initialized = False
        
        # 尝试初始化GPIO（最多重试5次）
        for attempt in range(5):
            try:
                self._init_gpio()
                self.gpio_initialized = True
                break
            except Exception as e:
                rospy.logwarn(f"GPIO初始化尝试 {attempt+1}/5 失败: {str(e)}")
                time.sleep(1)
        
        if not self.gpio_initialized:
            rospy.logerr("无法初始化GPIO，设备可能被占用")
            raise RuntimeError("GPIO初始化失败")

        # ROS发布器 - 发布接近开关状态
        self.all_inputs_pub = rospy.Publisher('proximity_sensor_data', Sensors, queue_size=1)
        
        # 定时器轮询读取GPIO状态
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.poll_sensors)
        
        rospy.loginfo("接近开关读取器初始化完成")

    def _init_gpio(self):
        """初始化GPIO引脚为输入模式并启用上拉电阻（参考你的命令：gpio mode 5 in + gpio mode 5 up）"""
        for sensor_name, pin in self.proximity_pins.items():
            try:
                # 设置引脚为输入模式
                subprocess.run(
                    ['gpio', 'mode', str(pin), 'in'],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                # 启用内部上拉电阻（和你的命令：gpio mode 5 up 一致）
                subprocess.run(
                    ['gpio', 'mode', str(pin), 'up'],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                rospy.loginfo(f"成功初始化 {sensor_name} (GPIO{pin}) 为输入模式+上拉电阻")
            except subprocess.CalledProcessError as e:
                rospy.logerr(f"初始化 {sensor_name} (GPIO{pin}) 失败: {e.stderr}")
                raise Exception(f"GPIO{pin} 初始化失败")

    def _read_gpio(self, pin):
        """读取单个GPIO引脚状态，返回布尔值（True=按下/触发，False=未触发）"""
        try:
            # 调用 gpio read 命令读取引脚值（返回 0/1）
            result = subprocess.run(
                ['gpio', 'read', str(pin)],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            # 解析输出（去除换行符）
            pin_value = int(result.stdout.strip())
            
            # 上拉模式下：0=触发（低电平），1=未触发（高电平），反转后更符合业务逻辑
            return pin_value == 0
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"读取GPIO{pin} 失败: {e.stderr}")
            return False
        except ValueError:
            rospy.logerr(f"解析GPIO{pin} 数值失败: {result.stdout}")
            return False

    def poll_sensors(self, event):
        """轮询读取所有接近开关状态并发布"""
        if not self.gpio_initialized:
            return
            
        try:
            # 读取两个接近开关状态（消抖：连续读取两次取稳定值）
            def read_stable(pin):
                val1 = self._read_gpio(pin)
                time.sleep(0.01)  # 10ms消抖
                val2 = self._read_gpio(pin)
                return val1 and val2  # 两次一致则确认状态
            
            # 读取传感器状态
            sensor_a = read_stable(self.proximity_pins['sensor_a'])
            sensor_b = read_stable(self.proximity_pins['sensor_b'])
            # 如需扩展sensor_c/sensor_d，添加对应读取逻辑
            # sensor_c = False  # 无则默认False
            # sensor_d = False  # 无则默认False

            # 封装并发布消息
            self._publish_sensor_data(sensor_a, sensor_b)
            
        except Exception as e:
            rospy.logerr(f"轮询传感器状态失败: {str(e)}")

    def _publish_sensor_data(self, sensor_a, sensor_b):
        """封装Sensors消息并发布"""
        try:
            msg = Sensors()
            msg.sensor_a = sensor_a
            msg.sensor_b = sensor_b
            # msg.sensor_c = sensor_c
            # msg.sensor_d = sensor_d
            
            self.all_inputs_pub.publish(msg)
            rospy.logdebug(f"发布接近开关状态: A={sensor_a}, B={sensor_b}")
        except Exception as e:
            rospy.logerr(f"发布传感器数据失败: {str(e)}")

    def cleanup(self):
        """安全清理GPIO资源"""
        if self.gpio_initialized:
            for sensor_name, pin in self.proximity_pins.items():
                try:
                    # 将引脚恢复为输入模式（可选）
                    subprocess.run(
                        ['gpio', 'mode', str(pin), 'in'],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    rospy.loginfo(f"已清理 {sensor_name} (GPIO{pin}) 资源")
                except Exception as e:
                    rospy.logerr(f"清理 {sensor_name} (GPIO{pin}) 失败: {str(e)}")
        rospy.loginfo("接近开关读取器已关闭")

if __name__ == '__main__':
    reader = None
    try:
        reader = ProximitySensorReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {str(e)}")
    finally:
        if reader:
            reader.cleanup()
        rospy.loginfo("接近开关读取器已退出")