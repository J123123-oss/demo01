import paho.mqtt.client as mqtt
import time
import rospy
from std_msgs.msg import String

class MQTTClient:
    def __init__(self, broker, port, topic_status, topic_cmd, client_id, ca_cert=None):
        """
        Initialize MQTT Client
        
        Args:
            broker (str): MQTT broker address
            port (int): MQTT broker port
            topic (str): Topic to subscribe/publish
            client_id (str): Client identifier
            ca_cert (str, optional): Path to CA certificate. Defaults to None.
        """
        self.broker = broker
        self.port = port
        self.topic_status = topic_status  # ROS状态发布到MQTT
        self.topic_cmd = topic_cmd        # MQTT控制指令下发到ROS
        self.client_id = client_id
        self.ca_cert = ca_cert
        self.ros_cmd_pub = None

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
        self.client.tls_set(
            ca_certs=self.ca_cert, 
            cert_reqs=mqtt.ssl.CERT_NONE  # Don't verify server cert (not recommended for production)
        )

    # =========================================================
    # MQTT Callback Methods (V2 version)
    # =========================================================
    def on_connect(self, client, userdata, flags, reason_code, properties):
            print(f"\n[状态] 服务器连接结果: {mqtt.connack_string(reason_code)}")
            if reason_code == mqtt.MQTT_ERR_SUCCESS:
                print(f"  ├─ 订阅主题: {self.topic_cmd}")
                client.subscribe(self.topic_cmd, qos=1)
                client.subscribe(self.topic_status, qos=1)

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        """Disconnection callback (V2 version)"""
        print(f"\n[状态] 与服务器断开连接: {mqtt.error_string(reason_code)}")
        print(f"  ├─ 标志: {disconnect_flags}")
        print(f"  └─ 原因代码: {reason_code}")

    def on_message(self, client, userdata, msg):
        """Message received callback"""
        print(f"\n[收到消息] \n  ├─ 主题: {msg.topic}\n  ├─ QoS: {msg.qos}\n  └─ 内容: {msg.payload.decode()}")
        # 只处理控制指令主题
        if msg.topic == self.topic_cmd and self.ros_cmd_pub:
            ros_msg = String()
            ros_msg.data = msg.payload.decode()
            self.ros_cmd_pub.publish(ros_msg)
            print(f"[MQTT->ROS] 已发布到 /robot_cmd: {ros_msg.data}")

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
    # Main Methods
    # =========================================================
    def connect(self):
        """Connect to MQTT broker"""
        keepalive = 60
        print(f"\n⏳ 连接到MQTT服务器: {self.broker}:{self.port} (TLS加密)...")
        
        try:
            self.client.connect(self.broker, self.port, keepalive)
            print(f"✅ 连接成功!")
            print(f"  ├─ 客户端ID: {self.client_id}")
            print(f"  └─ 测试主题: {self.topic_cmd}")
            print("=" * 50)
            return True
        except Exception as e:
            print(f"❌ 连接错误: {str(e)}")
            return False

    def start(self):
        """Start the MQTT client"""
        # 启动ROS节点
        rospy.init_node("mqtt_ros_bridge", anonymous=True)
        # 订阅robot_state话题，发布到MQTT状态主题
        rospy.Subscriber("robot_state", String, self.ros_robot_state_callback)
        self.ros_cmd_pub = rospy.Publisher("robot_cmd", String, queue_size=10)
        self.client.loop_start()
        try:
            print("🚀 运行中 (CTRL+C 退出)...")
            rospy.spin()  # 用spin替换死循环
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
    # Configuration
    config = {
        "broker": "129.211.16.114",
        "port": 8883,
        "topic_status": "robot/status",  # ROS状态发布到MQTT
        "topic_cmd": "robot/cmd",        # MQTT控制指令下发到ROS
        "client_id": "python-mqtt-client-v2",
        "ca_cert": None
    }
    
    # Create and start client
    mqtt_client = MQTTClient(**config)
    if mqtt_client.connect():
        mqtt_client.start()