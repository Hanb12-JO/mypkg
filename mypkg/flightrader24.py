import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
from bs4 import BeautifulSoup


class MostTrackedFlightNode(Node):
    def __init__(self):
        super().__init__('most_tracked_flight_node')
        self.pub = self.create_publisher(String, 'most_tracked_flight', 10)
        self.create_timer(60.0, self.get_most_tracked_flight)  # 1分ごとに実行

    def get_most_tracked_flight(self):
        try:
            # Flightradar24のウェブサイトにアクセス
            url = "https://www.flightradar24.com/data/most-tracked"
            response = requests.get(url)
            response.raise_for_status()

            # HTML解析
            soup = BeautifulSoup(response.text, 'html.parser')

            # 最も追跡されている航空機のデータを取得
            top_flight = soup.select_one('.most-tracked__flight-row')  # 適切なCSSセレクタに変更
            if top_flight:
                flight_number = top_flight.select_one('.flight-number').text.strip()
                airline = top_flight.select_one('.airline-name').text.strip()
                tracked_count = top_flight.select_one('.tracked-count').text.strip()

                # メッセージの作成
                message = f"Flight: {flight_number}, Airline: {airline}, Tracked: {tracked_count} times"
                self.pub.publish(String(data=message))
                self.get_logger().info(f"Published: {message}")
            else:
                self.get_logger().warning("No tracked flight data found.")
        except Exception as e:
            self.get_logger().error(f"Error fetching flight data: {e}")


def main():
    rclpy.init()

    node = MostTrackedFlightNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

