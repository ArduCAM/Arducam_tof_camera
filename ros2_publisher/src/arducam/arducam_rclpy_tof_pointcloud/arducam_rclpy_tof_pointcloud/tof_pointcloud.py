from math import tan, pi
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
from threading import Thread


from ArducamDepthCamera import (
    ArducamCamera,
    CameraInfo,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)


class TOFPublisher(Node):
    def __init__(self, tof: ArducamCamera, camera_info: CameraInfo):
        super().__init__("arducam")
        self.tof_ = tof
        if camera_info.device_type == TOFDeviceType.HQVGA:
            self.width_ = camera_info.width
            self.height_ = camera_info.height
        else:
            self.width_ = camera_info.width
            self.height_ = camera_info.height // 10 - 1
        self.pointsize_ = self.width_ * self.height_
        self.frame_id = "sensor_frame"
        self.depth_msg_ = Float32MultiArray()
        self.timer_ = self.create_timer(1 / 30, self.update)
        self.publisher_ = self.create_publisher(PointCloud2, "point_cloud", 10)
        self.publisher_depth_ = self.create_publisher(
            Float32MultiArray, "depth_frame", 10
        )
        self.fx = self.width_ / (2 * tan(0.5 * pi * 64.3 / 180))
        self.fy = self.height_ / (2 * tan(0.5 * pi * 50.4 / 180))
        self.header = Header()
        self.header.frame_id = "map"
        self.points = None
        self.running_ = True
        self.process_point_cloud_thr = Thread(
            target=self.__generateSensorPointCloud, daemon=True
        )
        self.process_point_cloud_thr.start()

    def __generateSensorPointCloud(self):
        while self.running_:
            frame = self.tof_.requestFrame(200)
            if frame is not None and isinstance(frame, DepthData):
                depth_buf = frame.depth_data
                confidence_buf = frame.confidence_data

                depth_buf[confidence_buf < 30] = 0

                self.depth_msg_.data = depth_buf.flatten() / 1000

                # Convert depth values ​​from millimeters to meters
                z = depth_buf / 1000.0
                z[z <= 0] = np.nan  # Handling invalid depth values

                # Calculate x and y coordinates
                u = np.arange(self.width_)
                v = np.arange(self.height_)
                u, v = np.meshgrid(u, v)

                # Calculate point cloud coordinates
                x = (u - self.width_ / 2) * z / self.fx
                y = (v - self.height_ / 2) * z / self.fy

                # Combined point cloud
                points = np.stack((x, y, z), axis=-1)
                self.points = points[
                    ~np.isnan(points).any(axis=-1)
                ]  # Filter invalid points

                self.tof_.releaseFrame(frame)

    def update(self):
        # self.__generateSensorPointCloud()
        if self.points is None:
            return
        self.header.stamp = self.get_clock().now().to_msg()

        pc2_msg_ = point_cloud2.create_cloud_xyz32(self.header, self.points)

        self.publisher_.publish(pc2_msg_)
        self.publisher_depth_.publish(self.depth_msg_)

    def stop(self):
        self.running_ = False
        self.process_point_cloud_thr.join()


def main(args=None):
    rclpy.init(args=args)
    tof = ArducamCamera()

    ret = 0
    ret = tof.open(Connection.CSI, 0)
    if not ret:
        print("Failed to open camera. Error code:", ret)
        return

    ret = tof.start(FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        tof.close()
        return

    info = tof.getCameraInfo()
    if info.device_type == DeviceType.HQVGA:
        tof.setControl(Control.RANGE, 4)

    print("pointcloud publisher start")

    tof_publisher = TOFPublisher(tof, info)

    rclpy.spin(tof_publisher)
    rclpy.shutdown()
    tof_publisher.stop()
    tof.stop()
    tof.close()


if __name__ == "__main__":
    main()
