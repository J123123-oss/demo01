import paho.mqtt.client as mqtt
import time

class MQTTClient:
    def __init__(self, broker, port, topic, client_id, ca_cert=None):
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
        self.topic = topic
        self.client_id = client_id
        self.ca_cert = ca_cert
        
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
        """Connection callback (V2 version)"""
        print(f"\n[状态] 服务器连接结果: {mqtt.connack_string(reason_code)}")
        if reason_code == mqtt.MQTT_ERR_SUCCESS:
            print(f"  ├─ 订阅主题: {self.topic}")
            client.subscribe(self.topic, qos=1)
            
            print(f"  └─ 发布测试消息到 {self.topic}")
            client.publish(self.topic, "Hello from Python MQTT client!", qos=1)

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        """Disconnection callback (V2 version)"""
        print(f"\n[状态] 与服务器断开连接: {mqtt.error_string(reason_code)}")
        print(f"  ├─ 标志: {disconnect_flags}")
        print(f"  └─ 原因代码: {reason_code}")

    def on_message(self, client, userdata, msg):
        """Message received callback"""
        print(f"\n[收到消息] \n  ├─ 主题: {msg.topic}\n  ├─ QoS: {msg.qos}\n  └─ 内容: {msg.payload.decode()}")

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
            print(f"  └─ 测试主题: {self.topic}")
            print("=" * 50)
            return True
        except Exception as e:
            print(f"❌ 连接错误: {str(e)}")
            return False

    def start(self):
        """Start the MQTT client"""
        self.client.loop_start()
        try:
            print("🚀 运行中 (CTRL+C 退出)...")
            while True:
                time.sleep(1)
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
    


if __name__ == "__main__":
    # Configuration
    config = {
        "broker": "129.211.16.114",
        "port": 8883,
        "topic": "test/topic",
        "client_id": "python-mqtt-client-v2",
        "ca_cert": None
    }
    
    # Create and start client
    mqtt_client = MQTTClient(**config)
    if mqtt_client.connect():
        mqtt_client.start()