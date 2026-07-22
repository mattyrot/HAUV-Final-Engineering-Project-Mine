#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

UBX_SYNC1 = 0xB5
UBX_SYNC2 = 0x62
NAV_PVT_CLASS = 0x01
NAV_PVT_ID    = 0x07
NAV_PVT_LEN   = 92

# fixType values from UBX-NAV-PVT
FIX_NO_FIX = 0
FIX_2D     = 2
FIX_3D     = 3
FIX_GNSS_DR = 4


def find_gps_port():
    for port in serial.tools.list_ports.comports():
        if 'u-blox' in port.description.lower() or 'ublox' in port.description.lower():
            return port.device
    for port in serial.tools.list_ports.comports():
        if 'ACM' in port.device or 'ttyUSB' in port.device:
            return port.device
    return None


def parse_nav_pvt(payload):
    """Parse UBX-NAV-PVT 92-byte payload."""
    if len(payload) < NAV_PVT_LEN:
        return None
    # offsets per u-blox M8 protocol spec
    fix_type = payload[20]
    flags    = payload[21]
    num_sv   = payload[23]
    lon      = struct.unpack_from('<i', payload, 24)[0] * 1e-7   # deg
    lat      = struct.unpack_from('<i', payload, 28)[0] * 1e-7   # deg
    h_msl    = struct.unpack_from('<i', payload, 36)[0] * 1e-3   # m above sea level
    h_acc    = struct.unpack_from('<I', payload, 40)[0] * 1e-3   # m (horizontal accuracy)

    gnss_fix_ok = bool(flags & 0x01)

    return {
        'lat': lat, 'lon': lon, 'alt': h_msl,
        'fix_type': fix_type, 'fix_ok': gnss_fix_ok,
        'num_sv': num_sv, 'h_acc': h_acc,
    }


class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        port = find_gps_port()
        if port is None:
            self.get_logger().error('No GPS serial port found!')
            raise RuntimeError('No GPS port')

        self.get_logger().info(f'GPS on {port} at 9600 baud (UBX NAV-PVT)')
        self.ser = serial.Serial(port, baudrate=9600, timeout=1)
        self._buf = bytearray()
        self.create_timer(0.05, self.timer_callback)  # 20 Hz poll

    def timer_callback(self):
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self._buf.extend(self.ser.read(waiting))
            self._parse_buf()
        except Exception as e:
            self.get_logger().error(f'GPS read error: {e}')

    def _parse_buf(self):
        buf = self._buf
        while len(buf) >= 8:
            # find sync
            if buf[0] != UBX_SYNC1 or buf[1] != UBX_SYNC2:
                del buf[0]
                continue

            cls = buf[2]
            mid = buf[3]
            length = struct.unpack_from('<H', buf, 4)[0]
            total = 6 + length + 2  # header + payload + checksum

            if len(buf) < total:
                break  # wait for more data

            payload = bytes(buf[6:6 + length])

            # verify checksum
            ck_a, ck_b = 0, 0
            for b in buf[2:6 + length]:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            if ck_a == buf[6 + length] and ck_b == buf[6 + length + 1]:
                if cls == NAV_PVT_CLASS and mid == NAV_PVT_ID:
                    self._publish_pvt(payload)

            del buf[:total]

    def _publish_pvt(self, payload):
        data = parse_nav_pvt(payload)
        if data is None:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        fix = data['fix_type']
        if fix in (FIX_3D, FIX_GNSS_DR) and data['fix_ok']:
            msg.status.status = NavSatStatus.STATUS_FIX
        elif fix == FIX_2D and data['fix_ok']:
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude  = data['lat']
        msg.longitude = data['lon']
        msg.altitude  = data['alt']

        h_acc = data['h_acc']
        msg.position_covariance[0] = h_acc ** 2
        msg.position_covariance[4] = h_acc ** 2
        msg.position_covariance[8] = (h_acc * 2) ** 2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'GPS fix={fix} sats={data["num_sv"]} '
            f'lat={data["lat"]:.6f} lon={data["lon"]:.6f} '
            f'alt={data["alt"]:.1f}m acc={data["h_acc"]:.1f}m'
        )

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
