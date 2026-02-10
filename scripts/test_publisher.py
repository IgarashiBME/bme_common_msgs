#!/usr/bin/env python3
"""
Test publisher for bme_common_msgs
Publishes all message types with dummy data for testing purposes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import math
import time

from bme_common_msgs.msg import AutoLog, GnssSolution, MavModes, PVT, RELPOSNED, HPPOSLLH, UTMHP


class TestPublisher(Node):
    def __init__(self):
        super().__init__('bme_common_msgs_test_publisher')

        # Create publishers for all message types
        self.pub_autolog = self.create_publisher(AutoLog, 'test/auto_log', 10)
        self.pub_mavmodes = self.create_publisher(MavModes, 'test/mav_modes', 10)
        self.pub_pvt = self.create_publisher(PVT, 'test/pvt', 10)
        self.pub_relposned = self.create_publisher(RELPOSNED, 'test/relposned', 10)
        self.pub_hpposllh = self.create_publisher(HPPOSLLH, 'test/hpposllh', 10)
        self.pub_utmhp = self.create_publisher(UTMHP, 'test/utmhp', 10)
        self.pub_gnss_solution = self.create_publisher(GnssSolution, 'test/gnss_solution', 10)

        # Timer for publishing at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

        self.get_logger().info('Test publisher started. Publishing on:')
        self.get_logger().info('  - test/auto_log')
        self.get_logger().info('  - test/mav_modes')
        self.get_logger().info('  - test/pvt')
        self.get_logger().info('  - test/relposned')
        self.get_logger().info('  - test/hpposllh')
        self.get_logger().info('  - test/utmhp')
        self.get_logger().info('  - test/gnss_solution')

    def make_header(self, frame_id='wgs84'):
        """Create a Header with current timestamp and specified frame_id."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def timer_callback(self):
        self.counter += 1
        itow = int(time.time() * 1000) % (7 * 24 * 3600 * 1000)  # GPS week time

        # Simulate moving position (around Tokyo)
        base_lat = 35.6812  # degrees
        base_lon = 139.7671  # degrees
        lat_offset = 0.0001 * math.sin(self.counter * 0.01)
        lon_offset = 0.0001 * math.cos(self.counter * 0.01)

        self.publish_autolog(itow)
        self.publish_mavmodes()
        self.publish_pvt(itow, base_lat + lat_offset, base_lon + lon_offset)
        self.publish_relposned(itow)
        self.publish_hpposllh(itow, base_lat + lat_offset, base_lon + lon_offset)
        self.publish_utmhp(itow, base_lat + lat_offset, base_lon + lon_offset)
        self.publish_gnss_solution(itow, base_lat + lat_offset, base_lon + lon_offset)

        if self.counter % 10 == 0:
            self.get_logger().info(f'Published {self.counter} messages')

    def publish_autolog(self, itow):
        msg = AutoLog()
        msg.stamp = self.get_clock().now().to_msg()
        msg.itow = float(itow)
        msg.rtk_status = 2  # Fixed
        msg.movingbase_status = 1
        msg.waypoint_seq = self.counter % 10
        msg.waypoint_start_x = 0.0
        msg.waypoint_start_y = 0.0
        msg.waypoint_end_x = 100.0
        msg.waypoint_end_y = 100.0
        msg.own_x = 50.0 + 10.0 * math.sin(self.counter * 0.05)
        msg.own_y = 50.0 + 10.0 * math.cos(self.counter * 0.05)
        msg.own_yaw = math.atan2(msg.own_y, msg.own_x)
        msg.tf_waypoint_x = msg.waypoint_end_x - msg.own_x
        msg.tf_waypoint_y = msg.waypoint_end_y - msg.own_y
        msg.tf_own_x = msg.own_x
        msg.tf_own_y = msg.own_y
        msg.cross_track_error = 0.5 * math.sin(self.counter * 0.02)
        msg.kp = 1.0
        msg.ki = 0.1
        msg.kd = 0.01
        msg.look_ahead_dist = 5.0
        msg.i_control_dist = 2.0
        msg.i_limit = 10.0
        msg.p = msg.kp * msg.cross_track_error
        msg.i = 0.0
        msg.d = 0.0
        msg.steering_ang = 5.0 * math.sin(self.counter * 0.03)
        msg.linear_x = 2.0
        msg.angular_z = 0.1 * math.sin(self.counter * 0.02)
        self.pub_autolog.publish(msg)

    def publish_mavmodes(self):
        msg = MavModes()
        msg.header = self.make_header('base_link')
        msg.base_mode = 209  # ARMED + GUIDED
        msg.custom_mode = 4  # Guided mode
        msg.mission_start = True
        self.pub_mavmodes.publish(msg)

    def publish_pvt(self, itow, lat, lon):
        msg = PVT()
        msg.header = self.make_header('wgs84')
        msg.itow = itow
        msg.year = 2024
        msg.month = 1
        msg.day = 15
        msg.hour = 12
        msg.min = 30
        msg.sec = self.counter % 60
        msg.valid = 0x07  # validDate, validTime, fullyResolved
        msg.t_acc = 50  # 50 ns
        msg.nano = 0
        msg.fix_type = 3  # 3D-fix
        msg.flags = 0xC1  # gnssFixOK, carrSoln=2 (fixed)
        msg.flags2 = 0xE0  # confirmedAvai, confirmedDate, confirmedTime
        msg.num_sv = 12
        msg.lon = int(lon * 1e7)
        msg.lat = int(lat * 1e7)
        msg.height = 50000  # 50m
        msg.h_msl = 35000  # 35m
        msg.h_acc = 10  # 10mm
        msg.v_acc = 15  # 15mm
        msg.vel_n = int(1000 * math.sin(self.counter * 0.01))  # mm/s
        msg.vel_e = int(1000 * math.cos(self.counter * 0.01))  # mm/s
        msg.vel_d = 0
        msg.g_speed = 1000  # 1 m/s
        msg.head_mot = int(math.degrees(math.atan2(msg.vel_e, msg.vel_n)) * 1e5)
        msg.s_acc = 50  # 50 mm/s
        msg.head_acc = 100000  # 1 deg
        msg.p_dop = 150  # 1.5
        msg.flags3 = 0x0000
        msg.head_veh = msg.head_mot
        msg.mag_dec = 0
        msg.mag_acc = 0
        msg.fix_status = 2  # Fixed
        msg.lon_deg = lon
        msg.lat_deg = lat
        self.pub_pvt.publish(msg)

    def publish_relposned(self, itow):
        msg = RELPOSNED()
        msg.header = self.make_header('base_station')
        msg.version = 1
        msg.ref_station_id = 1
        msg.itow = itow
        msg.rel_pos_n = int(100 * math.sin(self.counter * 0.01))  # cm
        msg.rel_pos_e = int(100 * math.cos(self.counter * 0.01))  # cm
        msg.rel_pos_d = 0
        msg.rel_pos_length = int(math.sqrt(msg.rel_pos_n**2 + msg.rel_pos_e**2))
        heading_rad = math.atan2(msg.rel_pos_e, msg.rel_pos_n)
        msg.rel_pos_heading = int(math.degrees(heading_rad) * 1e5)
        msg.rel_pos_hpn = 0
        msg.rel_pos_hpe = 0
        msg.rel_pos_hpd = 0
        msg.rel_pos_hp_length = 0
        msg.acc_n = 10  # 1mm
        msg.acc_e = 10
        msg.acc_d = 15
        msg.acc_length = 10
        msg.acc_heading = 50000  # 0.5 deg
        msg.flags = 0x01F7  # gnssFixOK, diffSoln, relPosValid, carrSoln=2, relPosHeadingValid
        msg.fix_status = 2  # Fixed
        msg.heading_rad = heading_rad
        msg.heading_deg = math.degrees(heading_rad)
        msg.qgc_heading = heading_rad
        self.pub_relposned.publish(msg)

    def publish_hpposllh(self, itow, lat, lon):
        msg = HPPOSLLH()
        msg.header = self.make_header('wgs84')
        msg.version = 0
        msg.flags = 0x00  # valid
        msg.itow = itow
        msg.lon = int(lon * 1e7)
        msg.lat = int(lat * 1e7)
        msg.height = 50000  # 50m in mm
        msg.h_msl = 35000  # 35m in mm
        msg.lon_hp = 50  # High precision component
        msg.lat_hp = 30
        msg.height_hp = 5
        msg.h_msl_hp = 3
        msg.h_acc = 100  # 10mm
        msg.v_acc = 150  # 15mm
        self.pub_hpposllh.publish(msg)

    def publish_utmhp(self, itow, lat, lon):
        msg = UTMHP()
        msg.header = self.make_header('utm')
        msg.itow = itow
        msg.num_sv = 12
        msg.fix_status = 2  # Fixed
        msg.lon_hp = lon
        msg.lat_hp = lat
        # Simple UTM conversion (approximate, zone 54N for Tokyo)
        msg.utm_easting = 500000.0 + (lon - 139.0) * 111320.0 * math.cos(math.radians(lat))
        msg.utm_northing = lat * 110540.0
        msg.height = 50.0  # 50m
        msg.h_acc = 10  # 10mm
        msg.v_acc = 15  # 15mm
        self.pub_utmhp.publish(msg)

    def publish_gnss_solution(self, itow, lat, lon):
        msg = GnssSolution()
        msg.header = self.make_header('wgs84')
        msg.itow = itow
        msg.num_sv = 12
        msg.position_rtk_status = 2  # Fixed
        msg.longitude = lon
        msg.latitude = lat
        # Simple UTM conversion (approximate, zone 54N for Tokyo)
        msg.utm_easting = 500000.0 + (lon - 139.0) * 111320.0 * math.cos(math.radians(lat))
        msg.utm_northing = lat * 110540.0
        msg.height = 50.0  # 50m
        msg.h_acc = 10  # 10mm
        msg.v_acc = 15  # 15mm
        msg.heading_rtk_status = 2  # Fixed
        heading_rad = math.atan2(
            math.sin(self.counter * 0.01),
            math.cos(self.counter * 0.01)
        )
        msg.heading_deg = math.degrees(heading_rad)
        self.pub_gnss_solution.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
