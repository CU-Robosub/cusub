#!/usr/bin/python3

import io
import sys
import time
import yaml
import threading

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped

import board
import busio

def initalizePressureSensor(i2c):

    result = bytearray(2)

    C1 = 0
    C2 = 0
    C3 = 0
    C4 = 0
    C5 = 0
    C6 = 0

    try:

        while not i2c.try_lock():
            # TODO timeout
            pass

        # Reset pressure sensor
        i2c.writeto(0x76, bytes([0x1E]), stop=True)

        # Wait for pressure sensor to be ready
        time.sleep(0.1)

        # Read PROM calibration data
        i2c.writeto(0x76, bytes([0xA2]), stop=False)
        i2c.readfrom_into(0x76, result)
        C1 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xA4]), stop=False)
        i2c.readfrom_into(0x76, result)
        C2 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xA6]), stop=False)
        i2c.readfrom_into(0x76, result)
        C3 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xA8]), stop=False)
        i2c.readfrom_into(0x76, result)
        C4 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xAA]), stop=False)
        i2c.readfrom_into(0x76, result)
        C5 = int.from_bytes(result, 'big', signed=False)

        i2c.writeto(0x76, bytes([0xAC]), stop=False)
        i2c.readfrom_into(0x76, result)
        C6 = int.from_bytes(result, 'big', signed=False)

        i2c.unlock()

    except OSError:
        # TODO print error
        print("OSERROR")
        pass
    except RuntimeError:
        # TODO print error
        print("RuntimeError")
        pass

    return [0, C1, C2, C3, C4, C5, C6]

def readAndCalibratePressure(i2c, C):

    result = bytearray(3)

    TEMP = 0
    P = 0

    try:

        while not i2c.try_lock():
            # TODO timeout
            pass

        # Start Pressure Conversion
        i2c.writeto(0x76, bytes([0x48]), stop=True)

        time.sleep(0.01)

        # Read Pressure Result
        i2c.writeto(0x76, bytes([0x00]), stop=False)
        i2c.readfrom_into(0x76, result)
        D1 = int.from_bytes(result, 'big', signed=False)

        # Start Temperature Conversion
        i2c.writeto(0x76, bytes([0x58]), stop=True)

        time.sleep(0.01)

        # Read Pressure Result
        i2c.writeto(0x76, bytes([0x00]), stop=False)
        i2c.readfrom_into(0x76, result)
        D2 = int.from_bytes(result, 'big', signed=False)

        dT = D2 - C[5]*256.0
        TEMP = 2000.0 + dT*C[6]/8388608.0

        OFF = C[2]*65536.0 + (C[4]*dT)/128.0
        SENS = C[1]*32768.0 + (C[3]*dT)/256.0
        P = (D1*SENS/2097152.0 - OFF)/8192.0

        i2c.unlock()

    except OSError as e:
        print("OSERROR2")
        print(str(e))
        return None, None
        pass
    except RuntimeError:
        print("Runtimeerror2")
        pass

    return TEMP, P

class DepthSensor():

    def __init__(self):
        print('initializing')

        # Load parameters
        subname = "triton" #TODO unhardcode

        # Initalise I2C
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Initalise Depth Sensor
        self.C = initalizePressureSensor(self.i2c)

        self.pub_odom = rospy.Publisher(
            'depth_odom', PoseWithCovarianceStamped, queue_size=1)
        self.pub_map = rospy.Publisher(
            'depth_map', PoseWithCovarianceStamped, queue_size=1)

        self.pub_odom_data = PoseWithCovarianceStamped()
        self.pub_odom_data.pose.covariance = [0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0]
        self.pub_odom_data.header.frame_id = subname + "/description/depth_odom_frame"

        self.pub_map_data = PoseWithCovarianceStamped()
        self.pub_map_data.pose.covariance = [0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0]
        self.pub_map_data.header.frame_id = subname + "/description/depth_map_frame"

        # Setup Depth calibrator
        depth_offset_data = None
        try:
            with open("depth_offset.yaml", "r") as stream:
                depth_offset_data = yaml.safe_load(stream)
                rospy.set_param('/triton/depth_offset', depth_offset_data['depth_offset'])
        except:
            rospy.logwarn("No depth saved!")

        # service to activate depth calibration
        #s = rospy.Service('calibrate_depth', CalibrateDepth, self.calibrate_depth)

        # Start Depth Publisher
        # depth_thread = threading.Thread(target = self.depth_publisher)
        # depth_thread.start()

        rospy.loginfo("Initalization Complete")
        self.depth_publisher()

        # Run until we kill the core
        # rospy.spin()

    def calibrate_depth(self, req):

        depth_offset = rospy.get_param('/triton/depth_offset', 0.0)

        requested_depth = req.depth.data
        depth_offset = requested_depth - self.current_depth + depth_offset
        rospy.set_param('/triton/depth_offset', depth_offset)
        rospy.loginfo("Setting depth_offset to: %f" % depth_offset)

        depth_offset_data = {'depth_offset': depth_offset}
        with io.open('depth_offset.yaml', 'w', encoding='utf8') as depth_offset_file:
            yaml.dump(depth_offset_data, depth_offset_file, default_flow_style=False, allow_unicode=True)

        return []


    def depth_publisher(self):

        r = rospy.Rate(10) # Depth 10Hz TODO pick good value
        x = 0
        while not rospy.is_shutdown():
            x += 1
            # print(x)

            # Publish Depth Data
            _, pressure = readAndCalibratePressure(self.i2c, self.C)
            if pressure is None:
                print('broken')
                # continue
                break
            pressure_mbar = pressure / 10.0
            depth_m = -1*(pressure_mbar-797.11)*100/(1030*9.8) + rospy.get_param('/triton/depth_offset', 0.0)

            self.current_depth = depth_m

            # rospy.loginfo("DEPTH %.2f %2.f" % (pressure_mbar, depth_m))

            # publish
            self.pub_odom_data.header.stamp = rospy.Time.now()
            self.pub_odom_data.header.seq += 1
            self.pub_odom_data.pose.pose.position.z = depth_m

            self.pub_map_data.header.stamp = rospy.Time.now()
            self.pub_map_data.header.seq += 1
            self.pub_map_data.pose.pose.position.z = depth_m

            self.pub_odom.publish(self.pub_odom_data)
            self.pub_map.publish(self.pub_map_data)

            r.sleep()


if __name__ == '__main__':

    rospy.init_node('depth_sensor')
    r = rospy.Rate(10) # Depth 10Hz TODO pick good value
    while not rospy.is_shutdown():
        try:
            node = DepthSensor()
        except rospy.ROSInterruptException:
            pass
        except:
            rospy.logwarn("An exception occured")
            r.sleep()