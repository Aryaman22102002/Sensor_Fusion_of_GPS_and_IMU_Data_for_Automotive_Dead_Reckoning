import rclpy
from rclpy.node import Node
import serial
import utm
import sys
from gps_msgs.msg import GPSmsg
from datetime import datetime, timedelta, timezone
from decimal import Decimal, getcontext

class GPSDriver(Node):
    def __init__(self, port):
        super().__init__('gps_driver')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.publisher_ = self.create_publisher(GPSmsg, '/gps', 10)
        self.frame_id = 'GPS1_Frame'
        self.port = port

        # this will check if the prot is avaliable or not, this is for debugging.
        try:
            self.serial_port = serial.Serial(self.port, baudrate=4800, timeout=1)
            self.get_logger().info(f"connected to GPS on prot :O {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"could not open serial port :( {self.port}: {e}")
            sys.exit(1)

        # sampling freq at 10 Hz (as mentioned, it will work best at 10hz)
        self.timer = self.create_timer(0.1, self.read_serial_data) 



    # our gps is giving ascii data so this will first read the data continously and decodes it from ascii
    def read_serial_data(self):
        try:
           
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('ascii', errors='replace').strip()
                self.get_logger().debug(f"raw NMEA sentence: {line}")
                self.process_line(line)
        except Exception as e:
            self.get_logger().error(f"error reading serial data :( {e}")

    def process_line(self, line):

        # we only need GPGGA, so this will allow GPGGA liines only

        if line.startswith('$GPGGA'):
            data = line.strip().split(',')

            # validate the data
            if len(data) < 15:
                self.get_logger().warn('incomplete data received..... :(')
                return

            #data[1] is UTC time (hhmmss.ss) in the GPGGA line, $GPGGA is data[0]

            time_str = data[1]
            new_time_str = Decimal(time_str)
            
            if new_time_str == '':
                timestamp = self.get_clock().now().to_msg()
            else:
                try:
                    hours = int(new_time_str // 10000) 
                    minutes = (new_time_str % 10000) // 100
                    
                    seconds = new_time_str % 100
                    
                    
                    #combining system date and gps time

                    now = datetime.utcnow().replace(tzinfo=timezone.utc)

                    gps_time = datetime(
                        year=now.year,
                        month=now.month,
                        day=now.day,
                        hour=hours,
                        minute=int(minutes),
                        second=int(seconds),
                        microsecond=int((seconds % 1) * int(Decimal(1e6))),
                        tzinfo=timezone.utc
                    )   
                   
                    #adding day rollover, it's important for sync. of gps time and system date. 

                    if gps_time > now + timedelta(hours=12):
                        gps_time -= timedelta(days=1)
                    elif gps_time < now - timedelta(hours=12):
                        gps_time += timedelta(days=1)
                    
                    getcontext().prec = 10
                    
                    secs = hours * 3600 + minutes * 60 + int(seconds)
                    nsecs = int((seconds - int(seconds)) * Decimal(1e9))
                    

                    # Calculate the seconds and nanoseconds for ROS time
                   # secs = int(gps_time.timestamp())
                   # nsecs = int((gps_time.timestamp() % 1) * 1e9)  # Fractional seconds converted to nanoseconds

                    # Convert to ROS Time message
                    timestamp = rclpy.time.Time(seconds=secs, nanoseconds=nsecs).to_msg()
                    
                    
                except ValueError:
                    self.get_logger().warn('invalide time in NMEA sentence')
                    timestamp = self.get_clock().now().to_msg()

            # lat format is ddmm.mmmm, N/S in the GPGGA
            
            lat_str = data[2]
            ns = data[3]
            if lat_str == '':
                lat = 0.0
            else:
                try:
                    lat_deg = float(lat_str[0:2])
                    lat_min = float(lat_str[2:])
                    lat = lat_deg + lat_min / 60.0
                    if ns == 'S':
                        lat = -lat # '-' sign because S is negative in lat
                except ValueError:
                    self.get_logger().error('error parsing lat')
                    return

            # long format same as lat
            
            lon_str = data[4]
            ew = data[5]
            if lon_str == '':
                long = 0.0
            else:
                try:
                    lon_deg = float(lon_str[0:3])
                    lon_min = float(lon_str[3:])
                    long = lon_deg + lon_min / 60.0
                    if ew == 'W':
                        long = -long # '-' sign because W is negative in lat
                except ValueError:
                    self.get_logger().error('error parsing long')
                    return

            # altitude
            try:
                altitude = float(data[9]) if data[9] else 0.0
            except ValueError:
                altitude = 0.0

            # lat long to umt
            try:
                utm_result = utm.from_latlon(lat, long) #magical line

                utm_easting = utm_result[0]
                utm_northing = utm_result[1]
                zone_number = utm_result[2]
                zone_letter = utm_result[3]
            except Exception as e:
                self.get_logger().error(f"error converting to utm: {e}")
                return


            #sending msgs using our gpsmsg package
            msg = GPSmsg()
            
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.latitude = lat
            msg.longitude = long
            msg.altitude = altitude
            msg.utm_easting = utm_easting
            msg.utm_northing = utm_northing
            msg.zone = zone_number
            msg.letter = zone_letter

            
            self.publisher_.publish(msg)
            self.get_logger().info(f'published GPS message: {msg}')
   

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("usage: driver.py [serial_port]")
        sys.exit(1)
    port = sys.argv[1]
    gps_driver = GPSDriver(port)
    rclpy.spin(gps_driver)
    gps_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
