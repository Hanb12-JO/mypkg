import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speedtest
import math
import threading


def bytes_to_mbps(size_bytes):
    """バイトをMbpsに変換"""
    return round(size_bytes / (1024 ** 2) * 8, 2)  # Bytes → Mbps変換


class WifiSpeedNode(Node):
    def __init__(self):
        super().__init__('wifi_speed_node')
        self.wifi = speedtest.Speedtest()
        self.pub = self.create_publisher(String, 'wifi_speed', 10)
        self.create_timer(1.0, self.measure_speed)  # 1秒間隔でタイマー起動
        self.lock = threading.Lock()  # スレッド安全のためのロック
        self.speed_data = {"download": 0.0, "upload": 0.0}  # 初期値

        # 初期設定でサーバーリストを取得し、最適なサーバーを選定
        self.get_logger().info("Configuring speedtest...")
        self.wifi.get_servers([])
        self.wifi.get_best_server()

    def measure_speed(self):
        # 別スレッドで速度測定を実行
        threading.Thread(target=self._perform_speedtest).start()

        # 最新の結果を取得してパブリッシュ
        with self.lock:
            download_speed_mbps = self.speed_data["download"]
            upload_speed_mbps = self.speed_data["upload"]

        # メッセージを作成してパブリッシュ
        message = f"Download: {download_speed_mbps} Mbps, Upload: {upload_speed_mbps} Mbps"
        self.pub.publish(String(data=message))
        self.get_logger().info(f"Published: {message}")

    def _perform_speedtest(self):
        try:
            # ダウンロードとアップロード速度を取得
            download_speed = self.wifi.download()
            upload_speed = self.wifi.upload()

            # 結果を保存
            with self.lock:
                self.speed_data["download"] = bytes_to_mbps(download_speed)
                self.speed_data["upload"] = bytes_to_mbps(upload_speed)

        except Exception as e:
            self.get_logger().error(f"Error during speedtest: {e}")


def main():
    rclpy.init()
    node = WifiSpeedNode()
    rclpy.spin(node)

