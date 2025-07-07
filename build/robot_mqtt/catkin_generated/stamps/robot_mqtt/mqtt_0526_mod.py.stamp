#!/usr/bin/env python3
import rospy
import json
import math
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist
from robot_localization.msg import INSPVAE,INSPVA, baseStatus, GPSData
from robot_control.msg import controlData  # 根据实际包名调整
import uuid
import time
class MQTTRobotBridge:
    def __init__(self):
        rospy.init_node('mqtt_robot_bridge', anonymous=True)
        
        # 初始化MQTT参数
        # self.mqtt_broker = rospy.get_param('~mqtt_broker', '106.12.23.8')
        # self.mqtt_port = rospy.get_param('~mqtt_port', 13234)
        # self.mqtt_broker = rospy.get_param('~mqtt_broker', 'broker.emqx.io')
        # self.mqtt_port = rospy.get_param('~mqtt_port', 1883)
        # self.mqtt_user = rospy.get_param('~mqtt_user', 'gifeng')
        # self.mqtt_password = rospy.get_param('~mqtt_password', '8dY9hE5FVF2GEIi')

        self.mqtt_broker = rospy.get_param('~mqtt_broker', 'pec81f9f.ala.cn-hangzhou.emqxsl.cn')
        self.mqtt_port = rospy.get_param('~mqtt_port', 8883)  # 使用8883端口用于TLS加密
        self.mqtt_tls = rospy.get_param('~mqtt_tls', True)  # 启用TLS
        self.mqtt_user = rospy.get_param('~mqtt_user', 'admin')
        self.mqtt_password = rospy.get_param('~mqtt_password', 'admin')
        # self.mqtt_protocol = rospy.get_param('~mqtt_protocol', 'mqtt')

        # self.mqtt_user = rospy.get_param('~mqtt_user', '123')
        # self.mqtt_password = rospy.get_param('~mqtt_password', '123')
        self.robot_id = rospy.get_param('~robot_id', 'GFSTJM120250201')
        self.pub_topic = rospy.get_param('~pub_topic', f'robot/{self.robot_id}/status')
        self.sub_topic = rospy.get_param('~sub_topic', 'robot/commands')
        self.uuid = str(uuid.uuid4())  # 生成唯一ID
        self.mqtt_connected = False
        self.low_8 = 0b00000000  # 低8位错误码初始化为0
          
        
        # 存储机器人状态数据
        self.robot_data = {
            # 固定帧头标识（需确认实际值）
            "header": "GIIFEN",
            
            # // Unix时间戳（单位：秒）
            "time_stamp": 1690000000,
            "uuid": self.uuid,
            
            # // 三轴加速度（单位：m/s²）
            "acceleration": {
                "x": 0.12,     
                "y": -0.05,    
                "z": 9.81      
            },
            
            # // GPS定位数据（需实际采集）
            "gps": {
                "latitude": 22.123456,   
                "longitude": 113.654321   
            },
            
            # // 三轴角速度（单位：°/s）
            "angular_velocity": {
                "x": 1.5,     
                "y": -0.3,     
                "z": 0.8       
            },
            
            # // 欧拉角姿态（单位：度） // 俯仰角（绕Y轴旋转）
            "pose": {
                "roll": 5.2,   
                "yaw": 12.7,   
                "pitch": -3.1  
            },
            
            # // 移动速度（单位：m/s，通过轮速计算）
            # // 换算公式：速度 = 面积 / 滚刷长度（0.62m）
            "speed": 0.5,
            "battery_voltage": 0,  # 电量
        
            # // 任务状态码（0:未开始，1:进行中）
            "task_status": 0,
            
            # // 手动录入的航线编号（字符串格式）
            "route_id": "RT001",
            "error": 0
            #bitmask
            }
        
        # 初始化ROS发布者和订阅者
        self.init_ros()
        
        # 初始化MQTT客户端
        self.mqtt_client = mqtt.Client(client_id=f"{self.robot_id}",
                                        callback_api_version=mqtt.CallbackAPIVersion.VERSION2 )
        # 启用TLS连接
        if self.mqtt_tls:
            self.mqtt_client.tls_set()  # 使用默认证书，如果有自定义证书，可以在此传入文件路径
            # self.mqtt_client.tls_insecure_set(True)  # 如果你不验证服务器证书（不推荐）  
        self.setup_mqtt()# 初始化ROS发布者和订阅者
        
        

    def init_ros(self):
        """初始化ROS组件"""
        # 订阅者
        # rospy.Subscriber("/inspvae_data", INSPVAE, self.inspvae_cb)
        rospy.Subscriber("/inspva_data", INSPVA, self.inspva_cb)
        rospy.Subscriber("/gps/raw", GPSData, self.drone_gps_cb)
        # rospy.Subscriber('/base_', Bool, self.task_callback)
        # rospy.Subscriber('/mission/route_id', String, self.route_callback)
        rospy.Subscriber("/base_status", baseStatus, self.base_cb)
        rospy.Subscriber('/control_data', String, self.control_callback)
        
        # 发布者（用于接收的MQTT消息）
        self.cmd_pub = rospy.Publisher('/mqtt_received', String, queue_size=10)

    def handle_error(self, error_message, error_code_binary):
        """处理错误日志并更新错误码"""
        # 将二进制错误码转换为十进制
        print(f"Binary Error Code: {error_code_binary}")
        # error_code_decimal = int(error_code_binary, 2)
        # rospy.logerr(f"{error_message} (Error Code: {error_code_decimal})")
        self.high_8_bits_value |= error_code_binary
        
        # 更新 robot_data 的错误码
        self.robot_data["error"] = (self.high_8_bits_value << 8) | self.low_8

    def setup_mqtt(self):
        """配置MQTT连接和回调"""
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        if self.mqtt_user is not None and self.mqtt_password is not None:
            self.mqtt_client.username_pw_set(self.mqtt_user, self.mqtt_password)
        
        while not self.mqtt_connected:
            try:
                self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
                self.mqtt_client.loop_start()
                rospy.loginfo(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
                self.mqtt_connected = True
            except Exception as e:
                # rospy.logerr(f"Initial MQTT connection failed: {str(e)}, retrying...")
                self.handle_error(f"Initial MQTT connection failed: {str(e)}, retrying...", "01000000")  

                time.sleep(5)  # 等待5秒后重试连接
            # rospy.signal_shutdown("MQTT connection error")

    # MQTT回调函数
    def on_mqtt_connect(self, client, userdata, flags, rc,properties=None):
        if rc == 0:
            rospy.loginfo("MQTT connected successfully")
            client.subscribe(self.sub_topic)
        else:
            # rospy.logerr(f"MQTT connection failed with code {rc}")
            self.handle_error(f"MQTT connection failed with {rc}", "00100000")  



    def on_mqtt_disconnect(self, client, userdata, disconnect_flags, rc, properties=None):
        rospy.logwarn(f"MQTT disconnected (rc={rc}), attempting reconnect...")
        self.mqtt_connected = False
        self.setup_mqtt()

    def control_callback(self, msg):
        if msg.robot_state != 1:
            self.robot_data["task_status"] = 0
        else:
            self.robot_data["task_status"] = 1

    # ROS回调函数on_m
    def inspvae_cb(self, msg):
        self.robot_data["gps"] = {
            "latitude": msg.latitude,
            "longitude": msg.longitude
        }
        self.robot_data["pose"] = {
            "roll": msg.roll,
            "pitch": msg.pitch,
            "yaw": msg.yaw
        }
        
        # self.robot_data["acceleration"] = {
        #     "x": msg.linear_acceleration.x,
        #     "y": msg.linear_acceleration.y,
        #     "z": msg.linear_acceleration.z
        # }

    def inspva_cb(self, msg):
        """处理IMU数据"""
        self.robot_data["gps"] = {
            "latitude": msg.latitude,
            "longitude": msg.longitude
        }
        self.robot_data["pose"] = {
            "roll": msg.roll,
            "pitch": msg.pitch,
            "yaw": msg.yaw
        }
        self.robot_data["acceleration"] = {
            "x": msg.acc_x,
            "y": msg.acc_y,
            "z": msg.acc_z
        }
        self.robot_data["angular_velocity"] = {
            "x": msg.gyro_x,
            "y": msg.gyro_y,
            "z": msg.gyro_z
        }


    def drone_gps_cb(self, msg):
        """处理无人机GPS数据"""
        # 处理GPS数据
        self.latitude_drone = msg.latitude
        self.longitude_drone = msg.longitude
        self.yaw_drone = msg.yaw


    def base_cb(self, msg):
        """处理基坐标系状态数据"""
        # 处理IMU数据
        self.speed = msg.speed
        self.distance_base = msg.distance
        self.sensor_state = msg.sensor_state
        self.complete_state = msg.complete_state
        self.rc_control = msg.rc_state
        self.robot_data['battery_voltage'] = msg.voltage # 电池电量(todo)
        self.robot_data['error']= msg.error

    def task_callback(self, msg):
        self.robot_data["task_started"] = msg.data

    def route_callback(self, msg):
        self.robot_data["route_id"] = msg.data

    def publish_robot_status(self):
        """发布机器人状态到MQTT"""
        try:
            self.robot_data["time_stamp"] = rospy.Time.now().to_sec()
            payload = json.dumps(self.robot_data)
            self.mqtt_client.publish(self.pub_topic, payload, qos=1)
            rospy.logdebug("Published to MQTT: %s", payload)
        except Exception as e:
            # rospy.logerr(f"MQTT publish error: {str(e)}")
            self.handle_error(f"MQTT publish error: {str(e)}", "00010000")  



    def run(self):
        rate = rospy.Rate(1/5)  # 发布频率5s一次
        while not rospy.is_shutdown():
            self.publish_robot_status()
            rate.sleep()

    def shutdown(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        rospy.loginfo("MQTT client disconnected")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            rospy.loginfo(f"Received MQTT message: {payload}")
            
            # 转发原始消息
            ros_msg = String()
            ros_msg.data = payload
            self.cmd_pub.publish(ros_msg)
            
            # JSON解析和命令处理
            self.process_command(payload)
            
        except Exception as e:
            rospy.logwarn(f"Error processing MQTT message: {str(e)}")

    def process_command(self, payload):
        """解析并执行JSON格式的命令"""
        try:
            command = json.loads(payload)
            
            # 基础命令校验
            if 'command' not in command:
                rospy.logwarn("Invalid command format: missing 'command' field")
                return
                
            cmd_type = command['command']
            
            # 根据命令类型路由处理
            if cmd_type == "control":
                self._handle_control_command(command)
            elif cmd_type == "mission":
                self._handle_mission_command(command)
            elif cmd_type == "system":
                self._handle_system_command(command)
            elif cmd_type == "route":
                self._handle_route_command(command)
            elif cmd_type == "newID":
                self._handle_newID_command(command)
            else:
                rospy.logwarn(f"Unknown command type: {cmd_type}")

        except json.JSONDecodeError:
            # rospy.logerr("Failed to parse JSON command")
            self.handle_error("Failed to parse JSON command", "00001000")  

        except KeyError as e:
            rospy.logwarn(f"Missing required field in command: {str(e)}")
        except Exception as e:
            # rospy.logerr(f"Error processing command: {str(e)}")
            self.handle_error(f"Error processing command: {str(e)}", "00000100")  

    
    def _handle_newID_command(self, command):
        """处理新的ID命令"""
        self.uuid = str(uuid.uuid4())
        self.robot_data["uuid"] = self.uuid


    def _handle_control_command(self, command):
        """处理速度控制命令"""
        
        # 创建ROS消息
        control = controlData()
        
        # 解析命令
        controlData.distance = command.get('distance', {})
        controlData.target_yaw = command.get('target_yaw', {})
        controlData.roller_speed = command.get('roller_speed', {})
        controlData.yaw = command.get('yaw', {})
        controlData.robot_state = command.get('robot_state', {})
        # 发布控制命令
        if not hasattr(self, 'control_data_pub'):
            self.control_data_pub = rospy.Publisher('/mqtt/control_data', controlData, queue_size=1)
        
        rospy.loginfo(f"Processed control command: {control}")

    def _handle_mission_command(self, command):
        """处理任务控制命令"""
        from std_msgs.msg import Bool
        from geometry_msgs.msg import PoseStamped
        
        action = command.get('action', '')
        
        if action == "start":
            # 示例：启动任务
            if 'target' in command:
                # 发布目标位置
                target = command['target']
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(target.get('x', 0.0))
                pose.pose.position.y = float(target.get('y', 0.0))
                
                if not hasattr(self, 'target_pub'):
                    self.target_pub = rospy.Publisher('/move_base_simple/goal', 
                                                    PoseStamped, 
                                                    queue_size=1)
                self.target_pub.publish(pose)
                
            # 发布任务开始信号
            start_msg = Bool(True)
            if not hasattr(self, 'mission_pub'):
                self.mission_pub = rospy.Publisher('/mission/start', Bool, queue_size=1)
            self.mission_pub.publish(start_msg)
            
        elif action == "stop":
            # 发布任务停止信号
            stop_msg = Bool(False)
            self.mission_pub.publish(stop_msg)
            
        else:
            rospy.logwarn(f"Unknown mission action: {action}")

    def _handle_route_command(self, command):
        """处理航线控制命令"""
        from std_msgs.msg import String
        
        route_id = command.get('route_id', '')
        
        if route_id:
            # 发布航线编号
            route_msg = String()
            route_msg.data = route_id
            self.robot_data["route_id"] = route_id
            
            rospy.loginfo(f"Published route ID: {route_id}")
        else:
            rospy.logwarn("Route ID is missing in the command")

    def _handle_system_command(self, command):
        """处理系统控制命令"""
        from std_srvs.srv import Trigger
        
        action = command.get('action', '')
        
        if action == "emergency_stop":
            try:
                emergency_srv = rospy.ServiceProxy('/emergency_stop', Trigger)
                response = emergency_srv()
                rospy.loginfo(f"Emergency stop response: {response.message}")
            except rospy.ServiceException as e:
                # rospy.logerr(f"Service call failed: {str(e)}")
                self.handle_error(f"Service call failed: {str(e)}", "00000010")  

        
        elif action == "reboot":
            try:
                reboot_srv = rospy.ServiceProxy('/reboot', Trigger)
                response = reboot_srv()
                rospy.loginfo(f"Reboot response: {response.message}")
            except rospy.ServiceException as e:
                # rospy.logerr(f"Service call failed: {str(e)}")
                self.handle_error(f"Service call failed: {str(e)}", "00000001")

            rospy.logwarn("Received reboot command - Implement actual reboot logic here")
            
        else:
            rospy.logwarn(f"Unknown system action: {action}")



if __name__ == '__main__':
    try:
        bridge = MQTTRobotBridge()
        rospy.on_shutdown(bridge.shutdown)
        bridge.run()
    except rospy.ROSInterruptException:
        pass