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
import os


def check_network(broker, timeout=3):
    """检测网络是否连通且MQTT Broker可达"""
    ping_param = "-n" if os.name == "nt" else "-c"
    ping_command = ["ping", ping_param, "1", "-W", str(timeout), broker]

    try:
        subprocess.run(
            ping_command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=timeout + 1
        )
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        public_dns = ["8.8.8.8", "114.114.114.114"]
        for dns in public_dns:
            try:
                subprocess.run(
                    ["ping", ping_param, "1", "-W", str(timeout), dns],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=True,
                    timeout=timeout + 1
                )
                rospy.logwarn(f"基础网络连通（DNS:{dns}），但MQTT Broker({broker})不可达")
                return False
            except:
                continue
        return False


class MQTTClient:
    def __init__(self, broker, port, user, password, topic_status, topic_cmd, topic_command, topic_result, client_id, ca_cert=None):
        self.broker = broker
        self.port = port
        self.user = user
        self.password = str(password)
        self.topic_status = topic_status  # ROS→MQTT 状态发布
        self.topic_cmd = topic_cmd        # MQTT→ROS 指令订阅
        self.topic_command = topic_command  # 终端命令订阅
        self.topic_result = topic_result    # 命令结果发布
        self.client_id = client_id
        self.ca_cert = ca_cert
        self.ros_cmd_pub = None
        self.lock = threading.Lock()

        # 创建客户端（V2 API）
        self.client = mqtt.Client(
            client_id=self.client_id, 
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )

        # 注册回调
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_publish = self.on_publish

        # TLS配置
        if self.ca_cert:
            self.client.tls_set(ca_certs=self.ca_cert, cert_reqs=mqtt.ssl.CERT_NONE)

    def on_connect(self, client, userdata, flags, reason_code, properties):
        """连接成功回调：订阅所有需要监听的MQTT话题"""
        rospy.loginfo(f"MQTT连接结果: {mqtt.connack_string(reason_code)}")
        if reason_code == mqtt.MQTT_ERR_SUCCESS:
            # 订阅指令话题（核心）
            subscribe_topics = [
                (self.topic_cmd, 1),        # MQTT→ROS 控制指令
                (self.topic_command, 1),    # 终端命令
                (self.topic_status, 1)      # 可选：如果需要监听自身发布的status话题
            ]
            # 批量订阅
            client.subscribe(subscribe_topics)
            for topic, qos in subscribe_topics:
                rospy.loginfo(f"已订阅MQTT话题: {topic} (QoS:{qos})")

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        rospy.logwarn(f"MQTT断开连接: {mqtt.error_string(reason_code)} (原因码:{reason_code})")

    def on_message(self, client, userdata, msg):
        """收到MQTT消息回调（核心处理逻辑）"""
        try:
            payload = msg.payload.decode('utf-8')
            rospy.loginfo(f"\n收到MQTT消息:\n  主题: {msg.topic}\n  QoS: {msg.qos}\n  内容: {payload}")

            # 处理控制指令（MQTT→ROS）
            if msg.topic == self.topic_cmd and self.ros_cmd_pub:
                try:
                    # 尝试解析JSON，确保格式合法
                    cmd_obj = json.loads(payload)
                    ros_msg = String(data=json.dumps(cmd_obj))
                except json.JSONDecodeError:
                    # 非JSON格式直接传递
                    ros_msg = String(data=payload)
                self.ros_cmd_pub.publish(ros_msg)
                rospy.loginfo(f"已转发MQTT消息到ROS话题 /robot_cmd: {ros_msg.data}")

            # 处理终端命令
            elif msg.topic == self.topic_command:
                self.handle_terminal_command(payload)

            # 可选：处理status话题的消息（如果需要监听）
            elif msg.topic == self.topic_status:
                rospy.loginfo(f"收到status话题消息: {payload}")

        except Exception as e:
            rospy.logerr(f"处理MQTT消息失败: {str(e)}\n{traceback.format_exc()}")

    def on_subscribe(self, client, userdata, mid, reason_codes, properties):
        """订阅成功回调"""
        rospy.loginfo(f"订阅成功 (消息ID: {mid})")

    def on_publish(self, client, userdata, mid, reason_code, properties):
        """发布成功回调"""
        rospy.loginfo(f"消息发布成功 (消息ID: {mid})")

    def handle_terminal_command(self, command_str):
        """处理终端命令并返回结果"""
        try:
            command_id = f"cmd_{int(time.time() * 1000)}"
            rospy.loginfo(f"执行终端命令: {command_str}")

            # 执行命令
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

            # 发布结果
            with self.lock:
                self.client.publish(
                    self.topic_result,
                    json.dumps(response, ensure_ascii=False),
                    qos=1
                )
            rospy.loginfo(f"已发送命令结果到 {self.topic_result}")

        except Exception as e:
            command_id = locals().get('command_id', f"cmd_err_{int(time.time()*1000)}")
            error_response = {
                "id": command_id,
                "command": command_str,
                "success": False,
                "error": str(e),
                "timestamp": time.time()
            }
            with self.lock:
                self.client.publish(self.topic_result, json.dumps(error_response), qos=1)
            rospy.logerr(f"命令执行失败: {str(e)}")

    def execute_command(self, command, timeout=60):
        """执行shell命令"""
        try:
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout, stderr = process.communicate(timeout=timeout)
            return {
                "stdout": stdout.strip(),
                "stderr": stderr.strip(),
                "returncode": process.returncode
            }
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate()
            return {
                "stdout": stdout.strip(),
                "stderr": f"超时({timeout}秒)\n{stderr.strip()}",
                "returncode": -1
            }
        except Exception as e:
            return {
                "stdout": "",
                "stderr": str(e),
                "returncode": -1
            }

    def connect(self):
        """连接MQTT服务器（带重试）"""
        while not check_network(self.broker):
            rospy.logwarn(f"网络不可用/Broker不可达，3秒后重试...")
            time.sleep(3)

        try:
            self.client.username_pw_set(self.user, self.password)
            self.client.reconnect_delay_set(min_delay=1, max_delay=60)
            self.client.connect(self.broker, self.port, 60)
            rospy.loginfo(f"MQTT连接成功！客户端ID: {self.client_id}")
            rospy.loginfo(f"状态发布话题: {self.topic_status}")
            rospy.loginfo(f"指令订阅话题: {self.topic_cmd}")
            return True
        except Exception as e:
            rospy.logerr(f"MQTT连接失败: {str(e)}，3秒后重试...")
            time.sleep(3)
            return self.connect()

    def start(self):
        """启动客户端"""
        # ROS话题订阅/发布
        rospy.Subscriber("robot_state", String, self.ros_robot_state_callback)
        self.ros_cmd_pub = rospy.Publisher("robot_cmd", String, queue_size=10)

        # 启动MQTT循环
        self.client.loop_start()
        try:
            rospy.loginfo("MQTT-ROS桥接已启动 (CTRL+C退出)")
            rospy.spin()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """停止客户端"""
        rospy.loginfo("断开MQTT连接...")
        self.client.disconnect()
        self.client.loop_stop()
        rospy.loginfo("MQTT连接已断开")

    def publish(self, topic, payload, qos=1):
        """发布MQTT消息"""
        with self.lock:
            self.client.publish(topic, payload, qos)

    def ros_robot_state_callback(self, msg):
        """ROS状态回调：发布到MQTT"""
        rospy.loginfo(f"收到ROS状态: {msg.data}")
        self.publish(self.topic_status, msg.data)


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("mqtt_ros_bridge", log_level=rospy.INFO)

    # 读取参数（增加日志调试）
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
    
    # 打印最终配置（关键调试）
    rospy.loginfo(f"最终加载的配置: {json.dumps(config, indent=2)}")

    # 启动客户端
    mqtt_client = MQTTClient(**config)
    if mqtt_client.connect():
        mqtt_client.start()