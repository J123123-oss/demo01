#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import time
import rospy
from std_msgs.msg import String
import json
import subprocess
import threading
import traceback
import os  # 新增：用于适配不同系统的ping命令参数


def check_network(broker, timeout=3):
    """
    检测网络是否连通且MQTT Broker可达
    Args:
        broker (str): MQTT Broker地址（IP或域名）
        timeout (int): 超时时间（秒）
    Returns:
        bool: 网络可用且Broker可达返回True，否则False
    """
    # 适配Windows/Linux/macOS的ping命令参数
    ping_param = "-n" if os.name == "nt" else "-c"  # Windows用-n，其他系统用-c
    # 构造ping命令：只发1个包，缩短检测时间
    ping_command = ["ping", ping_param, "1", "-W", str(timeout), broker]

    try:
        # 执行ping命令，不捕获输出（仅判断返回码）
        subprocess.run(
            ping_command,
            stdout=subprocess.DEVNULL,  # 屏蔽标准输出
            stderr=subprocess.DEVNULL,  # 屏蔽错误输出
            check=True,  # 返回码非0则抛异常
            timeout=timeout + 1  # 总超时时间（ping超时+命令执行缓冲）
        )
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        # ping Broker失败，进一步判断是无网络还是Broker不可达
        public_dns = ["8.8.8.8", "114.114.114.114"]  # 公共DNS（Google/国内）
        for dns in public_dns:
            try:
                subprocess.run(
                    ["ping", ping_param, "1", "-W", str(timeout), dns],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=True,
                    timeout=timeout + 1
                )
                # 能ping通DNS但ping不通Broker，说明基础网络通但Broker不可达
                print(f"[网络检测] 基础网络连通（DNS:{dns}），但MQTT Broker({broker})不可达")
                return False
            except:
                continue
        
        # 所有检测均失败，判定为无网络
        return False


class MQTTClient:
    def __init__(self, broker, port, user, password, topic_status, topic_cmd, topic_command, topic_result, client_id, ca_cert=None):
        """
        Initialize MQTT Client
        
        Args:
            broker (str): MQTT broker address
            port (int): MQTT broker port
            user (str): MQTT username
            password (str): MQTT password
            topic_status (str): ROS状态发布到MQTT
            topic_cmd (str): MQTT控制指令下发到ROS
            topic_command (str): 接收终端命令的主题
            topic_result (str): 发送命令结果的主题
            client_id (str): Client identifier
            ca_cert (str, optional): Path to CA certificate. Defaults to None.
        """
        self.broker = broker
        self.port = port
        self.user = user
        self.password = str(password)
        self.topic_status = topic_status  # ROS状态发布到MQTT
        self.topic_cmd = topic_cmd        # MQTT控制指令下发到ROS
        self.topic_command = topic_command  # 接收终端命令
        self.topic_result = topic_result    # 发送命令结果
        self.client_id = client_id
        self.ca_cert = ca_cert
        self.ros_cmd_pub = None

        # 线程安全锁
        self.lock = threading.Lock()

        # Create client instance (using V2 API)
        self.client = mqtt.Client(
            client_id=self.client_id, 
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )

        # Setup callbacks
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_publish = self.on_publish

        # Setup TLS
        if self.ca_cert:
            self.client.tls_set(
                ca_certs=self.ca_cert, 
                cert_reqs=mqtt.ssl.CERT_NONE
            )

    # =========================================================
    # MQTT Callback Methods (V2 version)
    # =========================================================
    def on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"\n[状态] 服务器连接结果: {mqtt.connack_string(reason_code)}")
        if reason_code == mqtt.MQTT_ERR_SUCCESS:
            print(f"  ├─ 订阅控制主题: {self.topic_cmd}")
            print(f"  ├─ 订阅命令主题: {self.topic_command}")
            client.subscribe(self.topic_cmd, qos=1)
            client.subscribe(self.topic_command, qos=1)

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        """Disconnection callback (V2 version)"""
        print(f"\n[状态] 与服务器断开连接: {mqtt.error_string(reason_code)}")
        print(f"  ├─ 标志: {disconnect_flags}")
        print(f"  └─ 原因代码: {reason_code}")

    def on_message(self, client, userdata, msg):
        """Message received callback"""
        print(f"\n[收到消息] \n  ├─ 主题: {msg.topic}\n  ├─ QoS: {msg.qos}\n  └─ 内容: {msg.payload.decode()}")
        
        try:
            # 处理控制指令主题 (ROS相关)
            if msg.topic == self.topic_cmd and self.ros_cmd_pub:
                ros_msg = String()
                try:
                    # 先解析验证是否是有效的 JSON
                    cmd_obj = json.loads(msg.payload.decode())
                    # 然后重新序列化确保格式正确
                    ros_msg.data = json.dumps(cmd_obj)
                except json.JSONDecodeError:
                    # 如果不是 JSON，按原样传递
                    ros_msg.data = msg.payload.decode()
                    
                self.ros_cmd_pub.publish(ros_msg)
                print(f"[MQTT->ROS] 已发布到 /robot_cmd: {ros_msg.data}")
            
            # 处理终端命令主题
            elif msg.topic == self.topic_command:
                self.handle_terminal_command(msg.payload.decode())
                
        except Exception as e:
            error_msg = f"处理消息时出错: {str(e)}\n{traceback.format_exc()}"
            print(error_msg)

    def on_subscribe(self, client, userdata, mid, reason_codes, properties):
        """Subscribe success callback (V2 version)"""
        if reason_codes and len(reason_codes) > 0:
            qos = reason_codes[0].value  # Get QoS of first subscription
            print(f"\n[状态] 订阅成功 (消息ID: {mid}, QoS: {qos})")
        else:
            print(f"\n[状态] 订阅成功 (消息ID: {mid})")

    def on_publish(self, client, userdata, mid, reason_code, properties):
        """Publish success callback (V2 version)"""
        print(f"\n[状态] 消息发布成功 (消息ID: {mid}, 原因代码: {reason_code})")

    # =========================================================
    # Terminal Command Handling
    # =========================================================
    def handle_terminal_command(self, command_str):
        """处理终端命令"""
        try:
            # 生成命令ID（使用时间戳）
            command_id = f"cmd_{int(time.time() * 1000)}"
            
            print(f"[命令执行] 开始执行命令: {command_str}")
            
            # 执行命令（默认超时60秒）
            result = self.execute_command(command_str, timeout=60)
            
            # 构建响应
            response = {
                "id": command_id,
                "command": command_str,
                "success": result["returncode"] == 0,
                "stdout": result["stdout"],
                "stderr": result["stderr"],
                "returncode": result["returncode"],
                "timestamp": time.time()
            }
            
            # 发送结果
            with self.lock:
                self.client.publish(
                    self.topic_result,
                    json.dumps(response, ensure_ascii=False),
                    qos=1
                )
                print(f"[命令执行] 已发送结果到 {self.topic_result}")
                
        except Exception as e:
            error_response = {
                "id": command_id if 'command_id' in locals() else f"cmd_err_{int(time.time()*1000)}",
                "command": command_str,
                "success": False,
                "error": str(e),
                "timestamp": time.time()
            }
            with self.lock:
                self.client.publish(
                    self.topic_result,
                    json.dumps(error_response, ensure_ascii=False),
                    qos=1
                )
            print(f"[命令执行] 错误: {str(e)}")

    def execute_command(self, command, timeout=60):
        """执行shell命令并返回结果"""
        try:
            print(f"[执行命令] 执行: {command}")
            print(f"[执行命令] 超时: {timeout}秒")
            
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                universal_newlines=True
            )
            
            # 使用communicate()获取输出并设置超时
            stdout, stderr = process.communicate(timeout=timeout)
            
            result = {
                "stdout": stdout.strip(),
                "stderr": stderr.strip(),
                "returncode": process.returncode
            }
            
            print(f"[执行命令] 完成, 返回码: {process.returncode}")
            if stdout.strip():
                print(f"[执行命令] stdout: {stdout.strip()}")
            if stderr.strip():
                print(f"[执行命令] stderr: {stderr.strip()}")
                
            return result
            
        except subprocess.TimeoutExpired:
            print(f"[执行命令] 超时, 终止进程")
            process.kill()
            stdout, stderr = process.communicate()
            return {
                "stdout": stdout.strip(),
                "stderr": f"命令执行超时 ({timeout}秒)\n{stderr.strip()}",
                "returncode": -1
            }
        except Exception as e:
            error_msg = f"执行命令时发生异常: {str(e)}"
            print(f"[执行命令] 异常: {error_msg}")
            return {
                "stdout": "",
                "stderr": error_msg,
                "returncode": -1
            }

    # =========================================================
    # Main Methods
    # =========================================================
    def connect(self):
        """Connect to MQTT broker (新增网络检测与自动重试)"""
        keepalive = 60
        # 1. 循环检测网络，直到网络可用且Broker可达
        while not check_network(self.broker):
            print(f"[网络检测] 网络不可用或Broker({self.broker})不可达，3秒后重试...")
            time.sleep(3)  # 每3秒重试一次，避免资源占用
        
        # 2. 网络可用，尝试连接MQTT Broker
        print(f"\n⏳ 网络已连通，尝试连接MQTT服务器: {self.broker}:{self.port} (TLS加密)...")
        
        try:
            # 设置用户名和密码
            self.client.username_pw_set(self.user, self.password)

            # 启用连接后自动重连（断网后会自动重试，间隔1-60秒）
            self.client.reconnect_delay_set(min_delay=1, max_delay=60)
        
            self.client.connect(self.broker, self.port, keepalive)
            print(f"✅ 连接成功!")
            print(f"  ├─ 客户端ID: {self.client_id}")
            print(f"  ├─ 控制主题: {self.topic_cmd} (ROS指令)")
            print(f"  ├─ 命令主题: {self.topic_command} (终端命令)")
            print(f"  ├─ 结果主题: {self.topic_result} (执行结果)")
            print(f"  └─ 状态主题: {self.topic_status} (ROS状态)")
            print("=" * 50)
            return True
        except Exception as e:
            # 连接Broker失败，重试（依托外层网络检测确保网络可用）
            print(f"❌ 连接Broker失败: {str(e)}，3秒后重试...")
            time.sleep(3)
            return self.connect()

    def start(self):
        """Start the MQTT client"""
        # 启动ROS节点订阅与发布
        rospy.Subscriber("robot_state", String, self.ros_robot_state_callback)
        self.ros_cmd_pub = rospy.Publisher("robot_cmd", String, queue_size=10)
        # 启动MQTT客户端循环
        self.client.loop_start()
        try:
            print("🚀 运行中 (CTRL+C 退出)...")
            print(f"📨 发送命令到: {self.topic_command}")
            print(f"📩 接收结果从: {self.topic_result}")
            rospy.spin()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Stop the MQTT client"""
        print("\n🛑 断开连接...")
        self.client.disconnect()
        self.client.loop_stop()
        print("✅ 已断开连接")

    def publish(self, topic, payload, qos=1):
        """Publish a message"""
        self.client.publish(topic, payload, qos)

    def ros_robot_state_callback(self, msg):
        """ROS回调：收到robot_state话题后发布到MQTT并打印"""
        print(f"[ROS] 收到robot_state: {msg.data}")
        # 只发布到MQTT状态主题
        self.publish(self.topic_status, msg.data)


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("mqtt_ros_bridge")  # 确保节点名称正确

    # 从ROS参数服务器读取配置（支持launch文件传参）
    config = {
        "broker": rospy.get_param("~broker", "121.40.57.48"),
        "port": int(rospy.get_param("~port", 1883)),
        "user": rospy.get_param("~user", "gf-mounted"),
        "password": rospy.get_param("~password", "20230810"),
        "topic_status": rospy.get_param("~topic_status", "robot/GF-HZ-TEST/status"),
        "topic_cmd": rospy.get_param("~topic_cmd", "robot/GF-HZ-TEST/cmd"),
        "topic_command": rospy.get_param("~topic_command", "robot/GF-HZ-TEST/command"),
        "topic_result": rospy.get_param("~topic_result", "robot/GF-HZ-TEST/result"),
        "client_id": rospy.get_param("~client_id", "python-mqtt-client-ID"),
        "ca_cert": None
    }
    
    rospy.loginfo(f"Final config: {config}")
    # 创建MQTT客户端并启动
    mqtt_client = MQTTClient(**config)
    if mqtt_client.connect():
        mqtt_client.start()