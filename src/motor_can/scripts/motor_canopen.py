import can
import time
import yaml
import rospy
import json
from std_msgs.msg import String
from serial_comms.msg import Distances
from serial_comms.msg import INSPVAE  # 确保导入正确的消息类型
import threading
import sys
import select
# import os

rate = 68  # Hz   166.66>> 68.26

class ServoDriveController:
    #def __init__(self, channel='vcan0', interface='socketcan'):
    def __init__(self, channel='can0', interface='socketcan'):
        self.bus = can.interface.Bus(channel=channel, interface=interface)
        #设置状态列表
        self.status_list = [
            "STOP",  # 停止状态
            "FORWARD",  # 前进状态
            "BACKWARD",  # 后退状态
        ]
        # 定义状态及其对应的速度配置
        self.status_config = {
            "STOP": {  # 停止状态
                "velocity_up": 0,
                "velocity_low": 0,
                "velocity_brush": 0
            },
            "FORWARD": {  # 前进状态
                "velocity_up": 250 * rate,
                "velocity_low": -250 * rate,
                "velocity_brush": -1500 * rate
            },
            "BACKWARD": {  # 后退状态
                "velocity_up": -250 * rate,
                "velocity_low": 250 * rate,
                "velocity_brush": 1500 * rate
            }
        }
        self.last_state = None  # 记录上一次的状态
        self.current_status = self.status_list[0]  # 当前默认停止状态
        self.current_velocity_up = 0   # ID = 3
        self.current_velocity_low = 0  # ID = 2
        self.current_velocity_brush = 0  # ID = 4
        self.last_left_speed = None
        self.last_right_speed = None
        self.last_brush_speed = None
        self.stop_flag = False   


        self.stop_velocity = 0  # 停止速度
        self.imu_yaw = 0.0  # IMU偏航角 单位度
        self.initial_yaw = None

        self.sensors_status = 0 #表示4个超声波传感器触发状态

        # PID参数
        self.pid_kp = 20.0
        self.pid_ki = 0.0
        self.pid_kd = 0.2
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.target_yaw = 0.0  # 期望偏航角（可根据需要设定）

        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)
        rospy.Subscriber('/robot_cmd', String, self.status_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.imu_callback)

    def set_state(self, new_state):
        if new_state not in self.status_config:
            rospy.logwarn(f"尝试设置无效状态: {new_state}")
            return False
        if new_state == self.current_status:
            return False  # 状态未改变
        self.current_status = new_state
        self.last_state = self.current_status
        rospy.loginfo(f"状态已更新为: {self.current_status}")
        return True

    # def set_state(self, new_state):
    #     """设置新状态并应用对应的速度配置"""
    #     correction = 0.0
    #     if new_state not in self.status_config:
    #         rospy.logwarn(f"尝试设置无效状态: {new_state}")
    #         return False

    #     if new_state == self.current_status:
    #         return False  # 状态未改变
    #     self.current_status = new_state
    #     config = self.status_config[new_state]

    #     # 设置速度
    #     self.current_velocity_up = config["velocity_up"]
    #     self.current_velocity_low = config["velocity_low"]
    #     self.current_velocity_brush = config["velocity_brush"]

    #     if self.imu_yaw and new_state in ["FORWARD", "BACKWARD"]:
    #         correction = self.pid_correction(self.imu_yaw)
    #         rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
    #         # 左右轮速度矫正（左轮-修正，右轮+修正，方向可根据实际调整）
    #     self.set_target_velocity(2, int(self.current_velocity_low - correction))
    #     self.set_target_velocity(3, int(self.current_velocity_up + correction))
    #     self.set_target_velocity(4, self.current_velocity_brush)
    #     # 发布状态
    #     self.publish_state()
    #     self.last_state = self.current_status
    #     rospy.loginfo(f"状态已更新为: {self.current_status}")
    #     return True

    def publish_state(self):
        """发布机器人状态信息，包含速度和状态"""
        state_msg = {
            "status": self.current_status,
            "velocity_up": self.current_velocity_up / rate,  # 单位转换为RPM
            "velocity_low": self.current_velocity_low / rate,
            "velocity_brush": self.current_velocity_brush / rate,
            "imu_yaw": self.imu_yaw,  # IMU偏航角
            "sensors_status": self.sensors_status,  # 超声波传感器状态
            "timestamp": time.time()
        }
        self.state_pub.publish(json.dumps(state_msg))

    def status_callback(self, msg):
        """处理状态消息"""
        self.set_state(msg.data)
        # if msg.data in self.status_list:
        #     self.current_status = msg.data
        #     if self.current_status == "STOP":
        #         self.current_velocity_up = self.stop_velocity
        #         self.current_velocity_low = self.stop_velocity
        #         self.current_velocity_brush = self.stop_velocity
        #         self.publish_state()
        #     elif self.current_status == "FORWARD":
        #         self.current_velocity_up = int(50 * rate)
        #         self.current_velocity_low = int(50 * rate)
        #         self.current_velocity_brush = int(3000 * rate)
        #         self.publish_state()
        #     elif self.current_status == "BACKWARD":
        #         self.current_velocity_up = int(-50 * rate)
        #         self.current_velocity_low = int(-50 * rate)
        #         self.current_velocity_brush = int(-3000 * rate)
        #         self.publish_state()
        #     else:
        #         rospy.logwarn(f"未知状态: {msg.data}")
        #     rospy.loginfo(f"当前状态更新为: {self.current_status}")
        # else:
        #     rospy.logwarn(f"收到未知状态: {msg.data}")
    def imu_callback(self, msg):
        """处理IMU数据"""
        try:
            self.imu_yaw = msg.yaw if hasattr(msg, "yaw") else msg.get("yaw", 0.0)
            # if self.imu_yaw > 180:
            #     self.imu_yaw -= 360
            if self.initial_yaw is None:
                self.initial_yaw = self.imu_yaw
                print(f"Initial IMU yaw set to: {self.initial_yaw} degrees")
            
            # 计算相对角度：将当前yaw值减去初始yaw值
            if self.initial_yaw is not None:
                relative_yaw = self.imu_yaw - self.initial_yaw

                # 处理yaw角度范围，确保在-180到180度之间
                if relative_yaw > 180:
                    relative_yaw -= 360
                elif relative_yaw < -180:
                    relative_yaw += 360

            self.imu_yaw = relative_yaw
            
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析IMU数据失败: {e}")

    def send_command(self, motor_id, command_data):
        frame_id = 0x600 + motor_id
        msg = can.Message(arbitration_id=frame_id, data=command_data, is_extended_id=False)
        self.bus.send(msg)
        time.sleep(0.05)
    
    def set_velocity_mode(self, motor_id):
        self.send_command(motor_id, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
        
    def set_target_velocity(self, motor_id, velocity):
        data = [
            0x23, 0xFF, 0x60, 0x00,
            velocity & 0xFF,
            (velocity >> 8) & 0xFF,
            (velocity >> 16) & 0xFF,
            (velocity >> 24) & 0xFF
        ]
        # print(f"设置电机 {motor_id} 目标速度: {velocity} RPM")
        self.send_command(motor_id, data)
    
    def set_acceleration(self, motor_id, acceleration):
        data = [
            0x23, 0x83, 0x60, 0x00,
            acceleration & 0xFF,
            (acceleration >> 8) & 0xFF,
            (acceleration >> 16) & 0xFF,
            (acceleration >> 24) & 0xFF
        ]
        self.send_command(motor_id, data)
    
    def set_deceleration(self, motor_id, deceleration):
        data = [
            0x23, 0x84, 0x60, 0x00,
            deceleration & 0xFF,
            (deceleration >> 8) & 0xFF,
            (deceleration >> 16) & 0xFF,
            (deceleration >> 24) & 0xFF
        ]
        self.send_command(motor_id, data)
    
    def enable_drive(self, motor_id):
        self.send_command(motor_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
    
    def start_motor(self, motor_id):
        if not 1 <= motor_id <= 127:
            raise ValueError(f"电机ID {motor_id} 超出有效范围 (1-127)")
        motor_id_byte = motor_id & 0xFF
        self.send_command(motor_id, [0x01, motor_id_byte])
    
    def configure_motor(self, motor_id, velocity, acceleration, deceleration):
        rospy.loginfo(f"配置电机 {motor_id}: 速度={int(velocity/rate)}, 加速度={acceleration}, 减速度={deceleration}")
        self.start_motor(motor_id)
        self.set_velocity_mode(motor_id)
        self.set_target_velocity(motor_id, velocity)  #输出转换为脉冲/秒
        self.set_acceleration(motor_id, acceleration)
        self.set_deceleration(motor_id, deceleration)
        self.enable_drive(motor_id)

    def shutdown(self):
        rospy.loginfo("正在关闭电机控制器...")
        self.set_target_velocity(2, 0)
        self.set_target_velocity(3, 0)
        self.set_target_velocity(4, 0)

        self.bus.shutdown()

    @staticmethod
    def load_config(config_file="/home/orangepi/demo01/src/motor_can/config/servo_config.yaml"):
    # def load_config(config_file="src/motor_can/config/servo_config.yaml"):
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
                return config
        except FileNotFoundError:
            rospy.logerr(f"错误: 配置文件 {config_file} 未找到")
            return {}
        except Exception as e:
            rospy.logerr(f"加载配置文件时出错: {e}")
            return {}
    
    def update_status_by_key(self, key):
        key_mapping = {
            's': "STOP",
            'f': "FORWARD",
            'b': "BACKWARD"
        }
        if key in key_mapping:
            self.set_state(key_mapping[key])
        else:
            rospy.loginfo(f"无效按键: {key}")
        # if key == 's':
        #     self.current_status = self.status_list[0]
        #     self.publish_state()
        #     rospy.loginfo("切换到停止状态")
        # elif key == 'f':
        #     self.current_status = self.status_list[1]
        #     self.publish_state()
        #     rospy.loginfo("切换到前进状态")
        # elif key == 'b':
        #     self.current_status = self.status_list[2]
        #     self.publish_state()
        #     rospy.loginfo("切换到后退状态")
        # else:
        #     rospy.loginfo("无效按键: %s" % key)

    def distance_callback(self, msg):
        """超声波距离检测回调"""
        if msg.distance_a < 30:
            self.sensors_status |= 0x01  # 设置传感器A状态
        else:
            self.sensors_status &= ~0x01
        if msg.distance_b < 30:
            self.sensors_status |= 0x02
        else:
            self.sensors_status &= ~0x02
        if msg.distance_c < 30:
            self.sensors_status |= 0x04
        else:
            self.sensors_status &= ~0x04
        if msg.distance_d < 30:
            self.sensors_status |= 0x08
        else:
            self.sensors_status &= ~0x08
        # 前进边缘检测
        # if not self.stop_flag and self.current_status == self.status_list[1]:  # FORWARD
        if self.current_status == self.status_list[1]:  # FORWARD
            if (msg.distance_a < 30 or msg.distance_b < 30):
                self.set_state("STOP")

        if self.current_status == self.status_list[2]:  # BACKWARD
            if (msg.distance_c < 30 or msg.distance_d < 30):
                self.set_state("STOP")

        # # 后退边缘检测
        # if not self.stop_flag and self.current_status == self.status_list[2]:  # BACKWARD
        #     if (msg.distance_c < 30 or msg.distance_d < 30):
        #         self.set_target_velocity(2, 0)
        #         self.set_target_velocity(3, 0)
        #         self.last_left_speed = 0
        #         self.last_right_speed = 0
        #         # self.current_velocity_brush = 0    #滚刷待定
        #         self.current_velocity_low = 0
        #         self.current_velocity_up = 0
        #         self.publish_state()
        #         self.stop_flag = True
        #     else:
        #         self.stop_flag = False


        # if (msg.distance_a < 30 or msg.distance_b < 30) and self.current_status == self.status_list[1]:
        #     # self.set_state("STOP")
        #     self.set_target_velocity(2, 0)
        #     self.set_target_velocity(3, 0)
        #     rospy.loginfo("正向超声波检测触发，暂停前进状态")
            # self.set_state("BACKWARD")

        # elif self.current_status == self.status_list[1]:
        #     self.set_state("FORWARD")
        #     rospy.loginfo("保持前进状态")

        # if (msg.distance_c < 30 or msg.distance_d < 30) and self.current_status == self.status_list[2]:
        #     self.set_target_velocity(2, 0)
        #     self.set_target_velocity(3, 0)
        #     rospy.loginfo("反向超声波检测触发，暂停后退状态")
        #     # self.set_state("FORWARD")
        # else:
        #     self.set_state("BACKWARD")
        #     rospy.loginfo("保持前进状态")

    def pid_correction(self, current_yaw):
        """根据IMU当前偏航角进行PID矫正，返回速度修正量"""
        error = self.target_yaw - current_yaw
        self.pid_integral += error
        derivative = error - self.pid_last_error
        correction = (self.pid_kp * error +
                      self.pid_ki * self.pid_integral +
                      self.pid_kd * derivative)
        self.pid_last_error = error
        return correction
    
    def execute_state(self, event=None):
        # 实时根据当前状态和IMU矫正左右轮速度
        if self.current_status in ["FORWARD", "BACKWARD"] and -15 < self.imu_yaw < 15:
            correction = self.pid_correction(self.imu_yaw)
            left_speed = int(self.status_config[self.current_status]["velocity_up"] - correction)
            right_speed = int(self.status_config[self.current_status]["velocity_low"] + correction)
            brush_speed = self.status_config[self.current_status]["velocity_brush"]
            rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
            # 左右轮速度矫正（左轮-修正，右轮+修正）
            if (self.last_left_speed != left_speed or
                self.last_right_speed != right_speed or
                self.last_brush_speed != brush_speed):
                rospy.loginfo(f"IMU矫正: yaw={self.imu_yaw:.2f}, correction={correction:.2f}")
                rospy.loginfo(f"左轮速度: {left_speed}, 右轮速度: {right_speed}")
                
                self.set_target_velocity(2, left_speed)
                self.set_target_velocity(3, right_speed)
                self.set_target_velocity(4, brush_speed)
                self.last_left_speed = left_speed
                self.last_right_speed = right_speed
                self.last_brush_speed = brush_speed
            # 实时发布状态
            self.current_velocity_low = left_speed
            self.current_velocity_up = right_speed
            self.current_velocity_brush = brush_speed
            # self.publish_state()
        elif self.current_status == "STOP" or not -15 < self.imu_yaw <15:
            if (self.last_left_speed != 0 or
                self.last_right_speed != 0 or
                self.last_brush_speed != 0):

                self.set_target_velocity(2, 0)
                self.set_target_velocity(3, 0)
                self.set_target_velocity(4, 0)
                self.last_left_speed = 0
                self.last_right_speed = 0
                self.last_brush_speed = 0

            self.current_velocity_low = 0
            self.current_velocity_up = 0
            self.current_velocity_brush = 0
            # self.publish_state()


    @staticmethod
    def keyboard_listener(controller):
        rospy.loginfo("按键控制：s=停止, f=前进, b=后退")
        while not rospy.is_shutdown():
            # 非阻塞读取键盘
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                controller.update_status_by_key(key)    

def main():
    rospy.init_node("motor_canopen_node")
    controller = ServoDriveController()
    config = controller.load_config()
    if not config or "motors" not in config or not config["motors"]:
        rospy.logerr("未找到有效配置，请检查配置文件")
        return
    rospy.loginfo("开始自动配置驱动器...")
    for motor in config["motors"]:
        motor_id = motor.get("id")
        velocity = motor.get("velocity")
        acceleration = motor.get("acceleration")
        deceleration = motor.get("deceleration")
        if None in (motor_id, velocity, acceleration, deceleration):
            rospy.logwarn(f"跳过无效配置: {motor}")
            continue
        try:
            rospy.loginfo(f"配置电机 {motor_id}...")
            controller.configure_motor(
                motor_id=motor_id,
                velocity=int(velocity*rate),
                acceleration=int(acceleration*rate),
                deceleration=int(deceleration*rate)
            )
            controller.current_status = controller.status_list[0]  # 初始化为停止状态
        except Exception as e:
            rospy.logerr(f"配置电机 {motor_id} 时出错: {e}")
    rospy.loginfo("电机初始化完成（Ctrl+C 退出）")
    # 订阅速度命令话题
    # rospy.Subscriber("distance_data", Distances, lambda msg: controller.distance_callback(msg))
    #通过检测按键修改运行状态
    # 启动键盘监听线程
    #t = threading.Thread(target=ServoDriveController.keyboard_listener, args=(controller,), daemon=True)
    #t.start()
    try:
    # 每0.5秒执行一次状态执行器
        rospy.Timer(rospy.Duration(0.5), controller.execute_state)
        # 每2秒发布一次状态
        rospy.Timer(rospy.Duration(0.5), lambda event: controller.publish_state())
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("程序终止")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()
