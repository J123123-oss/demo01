import can
import time
import yaml
import rospy
from std_msgs.msg import String
from serial_comms.msg import Distances
import threading
import sys
import select

rate = 166  # Hz

class ServoDriveController:
    def __init__(self, channel='vcan0', interface='socketcan'):
    # def __init__(self, channel='can0', interface='socketcan'):
        self.bus = can.interface.Bus(channel=channel, interface=interface)
        self.velocity_right_2 = 0
        self.velocity_left_3 = 0
        self.velocity_brush_4 = 0
        #设置状态列表
        self.status_list = [
            "STOP",  # 停止状态
            "FORWARD",  # 前进状态
            "BACKWARD",  # 后退状态
        ]
        self.last_state = None  # 记录上一次的状态
        self.current_status = self.status_list[0]  # 当前默认停止状态
        self.current_velocity_left = 0
        self.current_velocity_right = 0
        self.current_velocity_brush = 0
        self.stop_velocity = 0  # 停止速度
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)

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
    def load_config(config_file="/home/ubuntu/demo01/src/motor_can/config/servo_config.yaml"):
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
        if key == 's':
            self.current_status = self.status_list[0]
            self.state_pub.publish("STOP")
            rospy.loginfo("切换到停止状态")
        elif key == 'f':
            self.current_status = self.status_list[1]
            self.state_pub.publish("FORWARD")
            rospy.loginfo("切换到前进状态")
        elif key == 'b':
            self.current_status = self.status_list[2]
            self.state_pub.publish("BACKWARD")
            rospy.loginfo("切换到后退状态")
        else:
            rospy.loginfo("无效按键: %s" % key)

    def distance_callback(self, msg):
        # 仅在前进状态下判断距离
        if self.current_status == self.status_list[1] and msg.distance_a < 30:
            self.current_status = self.status_list[0]
            self.state_pub.publish("STOP")
            rospy.loginfo("超声波检测触发，切换到停止状态")
            self.last_state = self.current_status
        elif self.current_status == self.status_list[0] and msg.distance_a <= 30:
            # 条件满足时切换为前进  暂不可行
            self.current_status = self.status_list[1]
            self.state_pub.publish("FORWARD")
            rospy.loginfo("正常前进状态")
            self.last_state = self.current_status

        #判断后退到边缘状态
        if self.current_status == self.status_list[2] and msg.distance_a > 30:  # 如果当前状态是前进
            self.current_status = self.status_list[0]
            self.state_pub.publish("STOP")
            rospy.loginfo("超声波检测触发，切换到停止状态")
            self.last_state = self.current_status
        elif self.current_status == self.status_list[0] and msg.distance_a <= 30:
            # 条件满足时切换为前进  不可行
            self.current_status = self.status_list[2]
            self.state_pub.publish("FORWARD")
            rospy.loginfo("正常后退状态")
            self.last_state = self.current_status

    def execute_state(self, event=None):
        # 统一根据当前状态设置速度
        if self.last_state != self.current_status:
            if self.current_status == self.status_list[0]:  # STOP
                self.set_target_velocity(2, 0)
                self.set_target_velocity(3, 0)
                self.set_target_velocity(4, 0)
                self.last_state = self.current_status

            elif self.current_status == self.status_list[1]:  # FORWARD
                self.set_target_velocity(2, int(50*rate))
                self.set_target_velocity(3, int(50*rate))
                self.set_target_velocity(4, int(1200*rate))
                self.last_state = self.current_status

            elif self.current_status == self.status_list[2]:  # BACKWARD
                self.set_target_velocity(2, int(-50*rate))
                self.set_target_velocity(3, int(-50*rate))
                self.set_target_velocity(4, int(-1200*rate))
                self.last_state = self.current_status


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
    rospy.Subscriber("distance_data", Distances, lambda msg: controller.distance_callback(msg))
    #通过检测按键修改运行状态
    # 启动键盘监听线程
    t = threading.Thread(target=ServoDriveController.keyboard_listener, args=(controller,), daemon=True)
    t.start()
    try:
    # 每0.1秒执行一次状态执行器
        rospy.Timer(rospy.Duration(0.1), controller.execute_state)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("程序终止")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()