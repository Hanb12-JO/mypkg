import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speedtest
import math

### バイトからMbpsへの変換関数 ###
def bytes_to_mb(size_bytes):
    i = int(math.floor(math.log(size_bytes, 1024)))  # size_bytesを1024で対数をとる。KB → MB → GB
    power = math.pow(1024, i)  # pow(4,3)で4の3乗
    size = round(size_bytes / power, 2)  # 小数点以下を2桁に丸める。
    return f"{size} Mbps"

class WifiSpeedNode(Node):
    def __init__(self):
        super().__init__('wifispeed')
        self.wifi = speedtest.Speedtest()
        self.pub = self.create_publisher(String, 'wifi_speed', 10)
        self.create_timer(1.0, self.measure_speed) # タイマー

    def measure_speed(self):
        self.get_logger().info("Getting download speed...")
        download_speed = self.wifi.download()

        self.get_logger().info("Getting upload speed...")
        upload_speed = self.wifi.upload()

            ### Mbpsに変換 ###
        download_speed_mbps = bytes_to_mb(download_speed)
        upload_speed_mbps = bytes_to_mb(upload_speed)

            ### メッセージを作成してパブリッシュ###
        message = f"Download: {download_speed_mbps}, Upload: {upload_speed_mbps}"
        self.pub.publish(String(data=message))
        self.get_logger().info(f"Published: {message}")

def main():
    rclpy.init()
    node = WifiSpeedNode()
    rclpy.spin(node)

