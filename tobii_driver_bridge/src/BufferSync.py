import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

class BufferSync():
    """ Handles the buffer syncing
    "
    " When new eyetracking data arrives we store it in a et queue and the 
    " specific eyetracking to pts sync points in a et-pts-sync queue.
    "
    " When MPegTS is decoded we store pts values with the decoded offset in a
    " pts queue.
    "
    " When a frame is about to be displayed we take the offset and match with
    " the pts from the pts queue. This pts is then synced with the
    " et-pts-sync queue to get the current sync point. With this sync point
    " we can sync the et queue to the video output.
    "
    " For video frames without matching pts we move the eyetracking data forward
    " using the frame timestamps.
    " 
    """
    _et_syncs = []      # Eyetracking Sync items
    _et_queue = []      # Eyetracking Data items
    _et_ts = 0          # Last Eyetraing Timestamp
    _pts_queue = []     # Pts queue
    _pts_ts = 0         # Pts timestamp

    _gyro_queue = []
    _acc_queue = []

    _gyro = Vector3(0,0,0)
    _acc = Vector3(0,0,0)

    """initial parameters for Imu msgs"""
    # Liangzhe:TODO fill in undefined numbers
    _orientation = np.array([0., 0., 0., 1.])
    _orientation_covariance = [0.]*9
    _orientation_covariance[0] = -1

    def __init__(self, cb, pub):
        """ Initialize a Buffersync with a callback to call with each synced
        " eyetracking data
        """
        self._callback = cb
        self._imu_pub = pub
        self._imu_ready = False
        self._pickup_ts = 0
        self._ts_const = 0

        self._last_arriving_time = 0.
        self._counter = 0.
        self.period = 1

        sigma_g = rospy.get_param("~gyro_covariance", 0.0160447)
        sigma_a = rospy.get_param("~acc_covariance", 0.06084367)

        self._gyro_covariance = [sigma_g, 0., 0.,
                                 0., sigma_g, 0.,
                                 0., 0., sigma_g]
        self._acc_covariance = [sigma_a, 0., 0.,
                                0., sigma_a, 0.,
                                0., 0., sigma_a]

    def add_et(self, obj):
        """ Add new eyetracking data point """
        # Store sync (pts) objects in _sync queue instead of normal queue
        if 'pts' in obj:
            self._et_syncs.append(obj)
        elif 'gy' in obj:
            self._gyro_queue.append(obj)
        elif 'ac' in obj:
            self._acc_queue.append(obj)
        else:
            self._et_queue.append(obj)

    def sync_et(self, pts, timestamp):
        """ Sync eyetracking sync datapoints against a video pts timestamp and stores
        " the frame timestamp
        """
        # Split the gaze syncs to ones before pts and keep the ones after pts
        syncspast = filter(lambda x: x['pts'] <= pts, self._et_syncs)
        self._et_syncs = filter(lambda x: x['pts'] > pts, self._et_syncs)
        if syncspast != []:
            # store last sync time in gaze ts and video ts
            self._et_ts = syncspast[-1]['ts']
            self._pts_ts = timestamp

    def flush_et(self, timestamp):
        """ Flushes synced eyetracking on video
        " This calculates the current timestamp with the last gaze sync and the video
        " frame timestamp issuing a callback on eyetracking data that match the
        " timestamps.
        """
        nowts = self._et_ts + (timestamp - self._pts_ts)
        # Split the eyetracking queue into passed points and future points
        passed = filter(lambda x: x['ts'] <= nowts, self._et_queue)
        self._et_queue = filter(lambda x: x['ts'] > nowts, self._et_queue)
        # Send passed to callback
        self._callback(passed)

        if not self._imu_ready:
            self._pickup_ts = nowts
            self._imu_ready = True
        return nowts


    def add_pts_offset(self, offset, pts):
        """ Add pts to offset queue """
        self._pts_queue.append((offset, pts))

    def flush_pts(self, offset, timestamp):
        """ Flush pts to offset or use timestamp to move data forward """
        # Split pts queue to used offsets and past offsets
        used = filter(lambda x: x[0] <= offset, self._pts_queue)
        self._pts_queue = filter(lambda x: x[0] > offset, self._pts_queue)
        if used != []:
            # Sync with the last pts for this offset
            self.sync_et(used[-1][1], timestamp)
        nowts = self.flush_et(timestamp)
        return nowts

    def flush_imu_data(self, imu_rate, video_ready):
        """ Flush imu data and publish """
        if not self._imu_ready and not video_ready:
            self._ts_const = (int)(1e6/imu_rate)
            return

        # raise error
        if self._gyro_queue == []:
            rospy.logerr("Gyro queue exhausted!!!")
            self._pickup_ts += self._ts_const
            return
            #raise Exception("Gyro queue exhausted!!!")
        # raise error
        if self._acc_queue == []:
            rospy.logerr("Accelerometer queue exhausted!!!")
            self._pickup_ts += self._ts_const
            return
            #raise Exception("Accelerometer queue exhausted!!!")
            
        dequeue_gyro = filter(lambda x:x['ts'] <= self._pickup_ts, self._gyro_queue)
        dequeue_acc = filter(lambda x:x['ts'] <= self._pickup_ts, self._acc_queue)

        if dequeue_acc == []:
            if self._acc_queue[0]['ts'] - self._pickup_ts < 5000:
                dequeue_acc = [self._acc_queue[0]]
            else:
                rospy.logwarn("acc data packats has been dropped. queue size {}"\
                              .format(len(self._acc_queue)))
                self._pickup_ts += self._ts_const
                return
        if dequeue_gyro == []:
            if self._gyro_queue[0]['ts'] - self._pickup_ts < 5000:
                dequeue_gyro = [self._gyro_queue[0]]
            else:
                rospy.logwarn("gyro data packats has been dropped. queue size {}"\
                              .format(len(self._gyro_queue)))
                self._pickup_ts += self._ts_const
                return

        rospy.loginfo_throttle(5, "acc queue size:{}, gyro queue size:{}, time of delay {} ms"\
                       .format(len(self._acc_queue),len(self._gyro_queue), 
                               (self._acc_queue[-1]["ts"]-self._pickup_ts)/1000.))

        # update gyro queue
        self._gyro_queue = filter(lambda x:x['ts'] > self._pickup_ts, self._gyro_queue)
        # update acc queue
        self._acc_queue = filter(lambda x:x['ts'] > self._pickup_ts, self._acc_queue)

        pickup_acc = dequeue_acc[-1]
        pickup_gyro = dequeue_gyro[-1]

        self._gyro.x = pickup_gyro['gy'][0]*math.pi/180.
        self._gyro.y = pickup_gyro['gy'][1]*math.pi/180.
        self._gyro.z = pickup_gyro['gy'][2]*math.pi/180.

        self._acc.x = pickup_acc['ac'][0]
        self._acc.y = pickup_acc['ac'][1]
        self._acc.z = pickup_acc['ac'][2]

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.from_sec(self._pickup_ts/1e6)
        imu_msg.header.frame_id = "tobii_glasses"
        imu_msg.orientation = Quaternion(*self._orientation)
        imu_msg.orientation_covariance = self._orientation_covariance
        imu_msg.angular_velocity = self._gyro
        imu_msg.angular_velocity_covariance = self._gyro_covariance
        imu_msg.linear_acceleration = self._acc
        imu_msg.linear_acceleration_covariance = self._acc_covariance
        self._imu_pub.publish(imu_msg)

        self._pickup_ts += self._ts_const
