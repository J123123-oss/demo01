import paho.mqtt.client as mqtt
import rospy
import json
import time
import threading
import traceback
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RosMqttBridge:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("ros_mqtt_bridge", anonymous=True)
        
        # MQTT配置
        self.MQTT_BROKER = "129.211.16.114"
        self.MQTT_PORT = 8883
        self.MQTT_USER = None
        self.MQTT_PASS = None
        self.MQTT_TOPIC_PREFIX = "robot01/"
        self.ROBOT_ID = "robot01"
        
        # TLS配置
        self.CA_CERT = None  # "path/to/ca.crt"
        self.CLIENT_CERT = None  # "path/to/client.crt"
        self.CLIENT_KEY = None  # "path/to/client.key"
        
        # ROS与MQTT主题映射
        self.ROS_TO_MQTT = {
            "/chatter": f"{self.ROBOT_ID}/chatter",
            "/cmd_vel": f"{self.ROBOT_ID}/move"
        }
        
        self.MQTT_TO_ROS = {
            f"{self.ROBOT_ID}/control": "/control"
        }
        
        # MQTT客户端
        self.mqtt_client = mqtt.Client(
            client_id=f"ros_bridge_{self.ROBOT_ID}",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )
        
        # 设置自动重连策略
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)  # 指数退避重连
        
        # 设置回调函数
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        # ROS发布器
        self.ros_publishers = {}
        for mqtt_topic, ros_topic in self.MQTT_TO_ROS.items():
            # 创建对应ROS发布器
            self.ros_publishers[mqtt_topic] = rospy.Publisher(
                ros_topic,
                String,
                queue_size=10
            )
        
        # ROS订阅器
        for ros_topic, mqtt_topic in self.ROS_TO_MQTT.items():
            rospy.Subscriber(
                ros_topic, 
                rospy.AnyMsg, 
                lambda msg, topic=ros_topic: self._ros_callback(msg, topic)
            )
        
        # 连接标志和安全措施
        self.is_connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        
        # 启动MQTT连接
        self._connect_to_mqtt()
        
        rospy.loginfo("✅ ROS-MQTT桥接器初始化完成")
        rospy.loginfo(f"   ├─ MQTT服务器: {self.MQTT_BROKER}:{self.MQTT_PORT}")
        rospy.loginfo(f"   ├─ ROS监听: {list(self.ROS_TO_MQTT.keys())}")
        rospy.loginfo(f"   └─ MQTT监听: {list(self.MQTT_TO_ROS.keys())}")
        rospy.loginfo(f"   └─ 连接状态: {'已连接' if self.is_connected else '连接中...'}")
    
    def _connect_to_mqtt(self):
        """安全地连接MQTT服务器"""
        try:
            # 确保网络循环停止
            try:
                self.mqtt_client.loop_stop()
            except:
                pass
            
            # 设置TLS
            if self.CA_CERT or self.CLIENT_CERT:
                self.mqtt_client.tls_set(
                    ca_certs=self.CA_CERT,
                    # certfile=self.CLIENT_CERT,
                    # keyfile=self.CLIENT_KEY,
                    cert_reqs=mqtt.ssl.CERT_NONE
                    
                )
            
            # 用户名密码认证
            if self.MQTT_USER and self.MQTT_PASS:
                self.mqtt_client.username_pw_set(self.MQTT_USER, self.MQTT_PASS)
            
            rospy.loginfo(f"⏳ 连接到MQTT服务器: {self.MQTT_BROKER}:{self.MQTT_PORT}...")
            
            # 异步连接
            self.mqtt_client.connect_async(
                self.MQTT_BROKER,
                self.MQTT_PORT,
                60
            )
            
            # 启动网络循环
            self.mqtt_client.loop_start()
            while not rospy.is_shutdown() and not self.is_connected:
                self._connect_to_mqtt()
            
            # 设置连接超时监测
            # def connection_timeout_monitor():
            #     start_time = time.time()
            #     while not rospy.is_shutdown() and not self.is_connected:
            #         if time.time() - start_time > 10:  # 10秒超时
            #             rospy.logwarn("⚠️ 连接超时，尝试重新连接...")
            #             self.reconnect_attempts += 1
            #             if self.reconnect_attempts <= self.max_reconnect_attempts:
            #                 self._connect_to_mqtt()
            #             else:
            #                 rospy.logerr(f"❌ 已达到最大重连次数({self.max_reconnect_attempts})，请检查网络配置")
            #             break
            #         time.sleep(1)
            
            # threading.Thread(target=connection_timeout_monitor, daemon=True).start()
            
        except Exception as e:
            rospy.logerr(f"连接初始化失败: {e}")
            traceback.print_exc()
    
    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        """MQTT连接成功回调"""
        if reason_code == mqtt.MQTT_ERR_SUCCESS:
            self.is_connected = True
            self.reconnect_attempts = 0
            rospy.loginfo(f"✅ MQTT连接成功。订阅主题: {list(self.MQTT_TO_ROS.keys())}")
            
            # 订阅所有输入主题
            for topic in self.MQTT_TO_ROS.keys():
                client.subscribe(topic)
                rospy.loginfo(f"  ├─ 订阅: {topic}")
            
            rospy.loginfo("⏱️ 等待消息中...")
        else:
            self.is_connected = False
            conn_status = mqtt.connack_string(reason_code)
            rospy.logerr(f"❌ 连接失败: {conn_status}")
            
            # 尝试重新连接（如果超时）
            if reason_code == mqtt.CONNACK_REFUSED_SERVER_UNAVAILABLE:
                time.sleep(5)
                self._connect_to_mqtt()
    
    def _on_mqtt_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        """MQTT断开连接回调"""
        self.is_connected = False
        disconn_status = mqtt.error_string(reason_code)
        rospy.logerr(f"❌ 与服务器断开连接: {disconn_status}")
        
        # 避免过频繁的重连尝试
        reconnect_delay = min(2 ** self.reconnect_attempts, 30)  # 指数退避，最大30秒
        
        if self.reconnect_attempts < self.max_reconnect_attempts:
            rospy.loginfo(f"♻️ {reconnect_delay}秒后尝试重新连接...")
            time.sleep(reconnect_delay)
            self.reconnect_attempts += 1
            self._connect_to_mqtt()
        else:
            rospy.logerr(f"❌ 已达到最大重连次数({self.max_reconnect_attempts})，请检查网络配置")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT消息处理回调"""
        try:
            rospy.logdebug(f"📩 MQTT收到消息: [{msg.topic}] {str(msg.payload)[:50]}...")
            
            # 检查是否有对应ROS主题
            if msg.topic in self.MQTT_TO_ROS:
                ros_topic = self.MQTT_TO_ROS[msg.topic]
                
                try:
                    # JSON转ROS消息
                    try:
                        payload = json.loads(msg.payload)
                        ros_msg = String()
                        ros_msg.data = json.dumps(payload)  # 保持为JSON字符串
                    except:
                        ros_msg = String()
                        ros_msg.data = msg.payload.decode()
                    
                    # 发布到ROS
                    self.ros_publishers[msg.topic].publish(ros_msg)
                    rospy.loginfo(f"📤 MQTT→ROS: ({msg.topic} → {ros_topic}) {ros_msg.data[:60]}...")
                except Exception as e:
                    rospy.logerr(f"🚫 消息处理失败: {e}")
            else:
                rospy.logwarn(f"⚠️ 收到未处理主题: {msg.topic}")
        except Exception as e:
            rospy.logerr(f"🚫 MQTT消息处理异常: {e}")
            traceback.print_exc()
    
    def _ros_callback(self, msg, ros_topic):
        """ROS消息处理回调"""
        try:
            # 如果未连接到MQTT，跳过消息发送
            if not self.is_connected:
                rospy.logwarn_once("⚠️ MQTT未连接，跳过消息发送")
                return
                
            rospy.logdebug(f"📩 ROS收到消息: [{ros_topic}] {str(msg)[:50]}...")
            
            # 检查是否有对应MQTT主题
            if ros_topic in self.ROS_TO_MQTT:
                mqtt_topic = self.ROS_TO_MQTT[ros_topic]
                
                try:
                    # 转换消息为简单格式
                    if hasattr(msg, 'data'):
                        payload = msg.data
                    else:
                        # 转换ROS消息为字典
                        msg_dict = {}
                        for field in msg.__slots__:
                            if field != '_connection_header':
                                value = getattr(msg, field)
                                # 递归处理嵌套消息类型
                                if hasattr(value, '__slots__'):
                                    sub_msg_dict = {}
                                    for sub_field in value.__slots__:
                                        sub_value = getattr(value, sub_field)
                                        if sub_field != '_connection_header':
                                            sub_msg_dict[sub_field] = sub_value
                                    msg_dict[field] = sub_msg_dict
                                else:
                                    msg_dict[field] = value
                        payload = json.dumps(msg_dict)
                    
                    # 发布到MQTT
                    self.mqtt_client.publish(mqtt_topic, payload)
                    rospy.loginfo(f"📤 ROS→MQTT: ({ros_topic} → {mqtt_topic}) {str(payload)[:60]}...")
                except Exception as e:
                    rospy.logerr(f"🚫 消息转换失败: {e}")
            else:
                rospy.logwarn(f"⚠️ 收到未处理ROS话题: {ros_topic}")
        except Exception as e:
            rospy.logerr(f"🚫 ROS消息处理异常: {e}")
            traceback.print_exc()
    
    def run(self):
        """主运行循环"""
        # 状态监控线程
        def status_monitor():
            prev_status = self.is_connected
            while not rospy.is_shutdown():
                try:
                    # 连接状态变化时记录
                    if self.is_connected != prev_status:
                        prev_status = self.is_connected
                        rospy.loginfo(f"📡 MQTT连接状态: {'已连接 ✅' if self.is_connected else '已断开 ❌'}")
                    
                    time.sleep(1)
                except:
                    pass
                    
        threading.Thread(target=status_monitor, daemon=True).start()
        
        rospy.spin()  # 保持节点运行
        
        # 清理过程
        rospy.loginfo("🛑 节点正在关闭...")
        self.mqtt_client.disconnect()
        self.mqtt_client.loop_stop()

# =========================================================
# 主入口
# =========================================================
if __name__ == "__main__":
    try:
        rospy.loginfo("🚀 启动ROS-MQTT桥接器...")
        bridge = RosMqttBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"🚫 严重错误: {e}")
        traceback.print_exc()
