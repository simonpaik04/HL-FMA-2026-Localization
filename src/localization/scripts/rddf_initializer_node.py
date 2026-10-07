#!/usr/bin/python3
"""GPS/RViz로 선택한 RDDF 위치에서 IMU와 두 EKF를 한 번 초기화한다."""
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from robot_localization.srv import SetPose
from mando_localization.srv import SetInitialHeading
from rddf_initialization_core import RddfRouteMap

PREFIX = '/mando_localization/internal/initialization/'


class RddfInitializer:
    def __init__(self):
        self.p = rospy.get_param('~initialization')
        self.topics = rospy.get_param('~topics')
        self.frames = rospy.get_param('~frames')
        root = Path(rospy.get_param('~package_directory'))
        directory = Path(self.p['rddf_directory'])
        self.routes = RddfRouteMap(directory if directory.is_absolute() else root/directory)
        self.gps_route_name = self.p.get('gps_route_name')
        if self.gps_route_name is not None and self.gps_route_name not in self.routes.routes:
            raise ValueError('initialization/gps_route_name references an unknown route')
        reference = rospy.get_param('~reference')
        if (abs(reference['latitude_deg']-self.routes.origin['lat']) > 1e-10 or
                abs(reference['longitude_deg']-self.routes.origin['lng']) > 1e-10):
            raise ValueError('RDDF origin and GPS datum differ')
        for k,v in self.p.items():
            if k not in ('rddf_directory', 'gps_route_name') and (isinstance(v,bool) or not isinstance(v,(int,float)) or
                                         not math.isfinite(v) or v <= 0):
                raise ValueError('invalid initialization/'+k)
        self.lever = rospy.get_param('~lever_arm')
        self.quality = rospy.get_param('~quality')
        self.lock = threading.RLock()
        self.state, self.reason = 'WAITING_FOR_GPS', 'GPS 또는 시작 위치 선택 대기'
        self.selected = None
        self.manual = False
        self.ready = False
        self.speed = None
        self.clock = None
        self.last_gps_stamp = 0.
        self.gps_candidates = []
        self.epoch = 0
        self.last_now = None
        self.started = None
        self.confirm_after = None
        self.confirm = {'local': [], 'global': []}
        self.transaction = 0
        self.ready_pub = rospy.Publisher(PREFIX+'ready', Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(PREFIX+'status', String, queue_size=1, latch=True)
        self.commit_pub = rospy.Publisher(PREFIX+'committed_pose', PoseWithCovarianceStamped,
                                          queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber(self.topics['gps_fix'], NavSatFix, self.gps_callback, queue_size=10),
            rospy.Subscriber(self.topics['encoder_twist'], TwistWithCovarianceStamped, self.speed_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param('~internal_topics/clock_ready'), Bool, self.clock_callback, queue_size=2),
            rospy.Subscriber(PREFIX+'manual_request', String, self.manual_callback, queue_size=1),
            rospy.Subscriber(PREFIX+'manual_active', Bool, self.manual_active_callback, queue_size=1),
            rospy.Subscriber(self.topics['local_odometry'], Odometry, self.odom_callback, callback_args='local', queue_size=20),
            rospy.Subscriber(self.topics['global_odometry'], Odometry, self.odom_callback, callback_args='global', queue_size=20),
        ]
        self.timer = rospy.Timer(rospy.Duration(.1), self.tick, reset=True)
        self.publish()

    def publish(self):
        info = dict(state=self.state, reason=self.reason, ready=self.ready)
        if self.selected: info.update({k:v for k,v in self.selected.items() if k in ('source','route','index','x','y','yaw','distance')})
        self.ready_pub.publish(Bool(self.ready))
        self.status_pub.publish(String(json.dumps(info,ensure_ascii=False)))

    def tick(self, _):
        with self.lock:
            now = rospy.Time.now().to_sec()
            if self.last_now is not None and now < self.last_now-1e-6:
                self.epoch += 1
                self.ready = False
                self.manual = False
                self.selected = self.started = self.confirm_after = None
                self.confirm = {'local': [], 'global': []}
                self.gps_candidates.clear(); self.last_gps_stamp = 0.
                self.speed = self.clock = None
                self.state,self.reason = 'WAITING_FOR_GPS','ROS 시각 변경: 새 초기화 대기'
            self.last_now = now
            if self.started is not None and not self.ready and self.state != 'FAULT':
                if time.monotonic()-self.started > self.p['confirmation_timeout_sec']:
                    if self.confirm_after is not None:
                        self.reason = 'Local/Global 위치·yaw 확인 대기 (지연 중)'
            self.publish()

    def speed_callback(self,m):
        with self.lock:
            if m.header.frame_id==self.frames['base_link'] and math.isfinite(m.twist.twist.linear.x):
                self.speed=(m.header.stamp.to_sec(),m.twist.twist.linear.x,rospy.Time.now().to_sec(),time.monotonic())
            else: self.speed=None

    def clock_callback(self,m):
        with self.lock: self.clock=(bool(m.data),rospy.Time.now().to_sec(),time.monotonic())

    def stationary(self, now):
        return (self.speed is not None and 0 <= now-self.speed[2] <= self.p['speed_timeout_sec'] and
                -self.p['max_future_sec'] <= now-self.speed[0] <= self.p['speed_timeout_sec'] and
                time.monotonic()-self.speed[3] <= self.p['speed_timeout_sec'] and
                abs(self.speed[1]) <= self.p['max_speed_mps'])

    def clock_ready(self,now):
        return (self.clock is not None and self.clock[0] and
                0 <= now-self.clock[1] <= self.p['clock_timeout_sec'] and
                time.monotonic()-self.clock[2] <= self.p['clock_timeout_sec'])

    def manual_active_callback(self,m):
        with self.lock:
            if bool(m.data) and self.ready:
                self.ready = False
                self.selected = self.started = self.confirm_after = None
                self.confirm = {'local': [], 'global': []}
            elif self.started is not None:
                return
            self.manual=bool(m.data); self.gps_candidates.clear()
            self.state='WAITING_FOR_MANUAL' if self.manual else 'WAITING_FOR_GPS'
            self.reason='RDDF 위에 포인터를 놓고 클릭' if self.manual else 'GPS 대기'
            self.publish()

    def match(self,x,y,route=None,segment_index=None):
        return self.routes.match(x,y,self.p['max_snap_distance_m'],self.p['ambiguity_distance_m'],
                                 route_name=route,segment_index=segment_index)

    def gps_callback(self,m):
        with self.lock:
            if self.manual or self.ready or self.started is not None: return
            now=rospy.Time.now().to_sec(); stamp=m.header.stamp.to_sec()
            try:
                cov=np.asarray(m.position_covariance,dtype=float).reshape(3,3)
                known=m.position_covariance_type!=NavSatFix.COVARIANCE_TYPE_UNKNOWN
                valid=(m.position_covariance_type in (0,1,2,3) and m.status.status>=self.quality['minimum_fix_status'] and m.header.frame_id==self.frames['gps'] and
                       stamp>self.last_gps_stamp and stamp>0 and math.isfinite(m.altitude) and
                       -self.p['max_future_sec']<=now-stamp<=self.p['max_candidate_age_sec'] and
                       self.clock_ready(now))
                if known:
                    valid=valid and np.isfinite(cov).all() and np.allclose(cov,cov.T,atol=1e-8) and np.linalg.eigvalsh(cov).min()>=-1e-8 and all(0<=cov[i,i]<=self.quality['max_horizontal_variance_m2'] for i in (0,1)) and 0<=cov[2,2]<=self.quality['max_vertical_variance_m2']
                if not valid: raise ValueError('GPS 품질 또는 측정 시각 확인 대기')
                self.last_gps_stamp=stamp
                x,y=self.routes.project_gps(m.latitude,m.longitude)
                candidate=self.match(x,y,self.gps_route_name)
                if candidate['accepted']:
                    yaw=candidate['yaw'];lx,ly=self.lever['x_m'],self.lever['y_m']
                    candidate=self.match(x-math.cos(yaw)*lx+math.sin(yaw)*ly,
                                         y-math.sin(yaw)*lx-math.cos(yaw)*ly,candidate['route'])
                if not candidate['accepted']: raise ValueError(candidate['reason'])
            except (ValueError,TypeError,KeyError,np.linalg.LinAlgError) as error:
                self.gps_candidates.clear();self.state='WAITING_FOR_GPS';self.reason=str(error);return
            candidate.update(source='GPS_RDDF',stamp=stamp)
            if self.gps_candidates:
                first=self.gps_candidates[0]
                if (candidate['route']!=first['route'] or stamp-self.gps_candidates[-1]['stamp']>self.p['max_candidate_age_sec'] or
                    math.hypot(candidate['x']-first['x'],candidate['y']-first['y'])>self.p['gps_cluster_radius_m']):
                    self.gps_candidates.clear()
            self.gps_candidates.append(candidate)
            self.gps_candidates=self.gps_candidates[-int(self.p["gps_consecutive_samples"]):]
            if len(self.gps_candidates)<self.p['gps_consecutive_samples']:
                self.reason='안정된 GPS 후보 수집';return
            self.begin(candidate,now)

    def manual_callback(self,m):
        with self.lock:
            if self.ready or self.started is not None: return
            try:
                data=json.loads(m.data);now=rospy.Time.now().to_sec()
                if data['frame_id']!=self.frames['map'] or not 0<=now-float(data['stamp'])<=self.p['max_candidate_age_sec']:
                    raise ValueError('수동 선택 frame 또는 시각 오류')
                candidate=self.match(float(data['x']),float(data['y']),data['route'],data.get('index'))
                if not candidate['accepted']:raise ValueError(candidate['reason'])
                # Heading is always recalculated from RDDF; the mouse never supplies arbitrary yaw.
                candidate.update(source='MANUAL_RDDF',stamp=float(data['stamp']))
                self.manual=True
                self.begin(candidate,now)
            except (ValueError,TypeError,KeyError) as error:
                self.state,self.reason='WAITING_FOR_MANUAL',str(error)
                self.publish()

    def pose(self,target,stamp,frame):
        m=PoseWithCovarianceStamped();m.header.stamp=stamp;m.header.frame_id=frame
        m.pose.pose.position.x=target['x'];m.pose.pose.position.y=target['y']
        m.pose.pose.orientation.z=math.sin(target['yaw']/2);m.pose.pose.orientation.w=math.cos(target['yaw']/2)
        for i in (0,7):m.pose.covariance[i]=self.p['position_variance_m2']
        for i in (14,21,28):m.pose.covariance[i]=1e6
        m.pose.covariance[35]=math.radians(self.p['heading_standard_deviation_deg'])**2
        return m

    def begin(self,candidate,now):
        if candidate.get('source') != 'MANUAL_RDDF' and not self.stationary(now):
            self.state,self.reason='WAITING_FOR_STATIONARY','fresh 엔코더 정지 확인 대기';return
        self.selected=dict(candidate);self.started=time.monotonic();self.state='INITIALIZING';self.reason='IMU와 Local/Global 초기화'
        self.transaction+=1;epoch=self.epoch;transaction=self.transaction
        self.publish()
        threading.Thread(target=self.initialize,args=(dict(candidate),epoch,transaction),daemon=True).start()

    def initialize(self,target,epoch,transaction):
        try:
            heading_name=rospy.get_param('~heading_service','/calibrated_imu/set_initial_heading')
            local_name=rospy.get_param('~services/local_ekf_set_pose')
            global_name=rospy.get_param('~services/global_ekf_set_pose')
            # Availability checks precede all mutations. Calls run outside callbacks so heartbeat remains false.
            for service in (heading_name,local_name,global_name):
                while True:
                    with self.lock:
                        if epoch!=self.epoch or self.state=='FAULT':return
                    try:
                        rospy.wait_for_service(service,timeout=self.p['service_wait_sec']);break
                    except rospy.ROSException:
                        if rospy.is_shutdown():return
                        with self.lock:
                            if epoch!=self.epoch:return
                            self.reason='초기화 서비스 준비 대기: '+service

            with self.lock:
                if (epoch!=self.epoch or
                        target.get('source') != 'MANUAL_RDDF' and not self.stationary(rospy.Time.now().to_sec())):
                    raise ValueError('초기화 전 차량 상태 변경')
            while True:
                with self.lock:
                    if (epoch!=self.epoch or self.state=='FAULT' or
                            target.get('source') != 'MANUAL_RDDF' and not self.stationary(rospy.Time.now().to_sec())):
                        raise ValueError('초기화 대기 중 정지/시간 조건 변경')
                result=rospy.ServiceProxy(heading_name,SetInitialHeading)(transaction,target['yaw'],target['source']+':'+target['route']+':'+str(target['index']),self.p['heading_standard_deviation_deg'])
                if result.accepted:break
                if result.reason!='WAITING_FOR_FRESH_IMU':raise ValueError(result.reason)
                with self.lock:self.state,self.reason='WAITING_FOR_IMU','fresh IMU와 장착 TF 대기'
                time.sleep(.03)
            with self.lock:
                if epoch!=self.epoch:return
                self.state='INITIALIZING'

            for name,frame in ((local_name,self.frames['odom']),(global_name,self.frames['map'])):
                with self.lock:
                    if (epoch!=self.epoch or
                            target.get('source') != 'MANUAL_RDDF' and not self.stationary(rospy.Time.now().to_sec())):
                        raise ValueError('초기화 중 차량 상태 변경')
                rospy.ServiceProxy(name,SetPose)(self.pose(target,rospy.Time.now(),frame))
            with self.lock:
                if epoch!=self.epoch:return
                self.confirm_after=rospy.Time.now().to_sec();self.reason='Local/Global 위치·yaw 확인 대기'
        except (rospy.ROSException,rospy.ServiceException,ValueError) as error:
            with self.lock:
                if epoch==self.epoch:self.state,self.reason='FAULT',str(error)

    def odom_callback(self,m,key):
        with self.lock:
            if self.confirm_after is None or self.ready or self.state=='FAULT':return
            now=rospy.Time.now().to_sec();stamp=m.header.stamp.to_sec()
            if self.selected.get('source') != 'MANUAL_RDDF' and not self.stationary(now):
                self.state,self.reason='FAULT','초기화 확인 중 차량 이동';return
            q=m.pose.pose.orientation;p=m.pose.pose.position
            values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]+list(m.pose.covariance)
            norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            if not all(math.isfinite(v) for v in values) or abs(norm-1)>1e-3:return
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            angle=math.atan2(math.sin(yaw-self.selected['yaw']),math.cos(yaw-self.selected['yaw']))
            okay=(m.header.frame_id==self.frames['odom' if key=='local' else 'map'] and m.child_frame_id==self.frames['base_link'] and stamp>self.confirm_after and
                  -self.p['max_future_sec']<=now-stamp<=self.p['speed_timeout_sec'] and
                  math.hypot(p.x-self.selected['x'],p.y-self.selected['y'])<=self.p['confirmation_distance_m'] and abs(angle)<=math.radians(self.p['confirmation_yaw_deg']))
            if not okay:self.confirm[key].clear();return
            if not self.confirm[key] or stamp>self.confirm[key][-1]:self.confirm[key].append(stamp)
            if all(len(v)>=self.p['confirmation_samples'] and now-v[-1]<=self.p['speed_timeout_sec'] for v in self.confirm.values()):
                self.ready=True;self.state='READY';self.reason='RDDF 초기 위치·방향 적용 확인'
                self.commit_pub.publish(self.pose(self.selected,rospy.Time.now(),self.frames['map']))
                self.publish()


if __name__=='__main__':
    rospy.init_node('rddf_initializer')
    try:node=RddfInitializer();rospy.spin()
    except (ValueError,KeyError,OSError) as error:rospy.logfatal('RDDF initialization failed: %s',error);raise
