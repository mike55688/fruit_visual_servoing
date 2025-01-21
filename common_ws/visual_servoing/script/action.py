#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from geometry_msgs.msg import Twist
from forklift_driver.msg import Meteorcar
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import qos_profile_sensor_data
import math
import rclpy.time
import tf_transformations
import rclpy
import rclpy.logging
from rclpy.node import Node
from enum import Enum
import time
import statistics
from rclpy.clock import Clock
from dataclasses import dataclass

def fnCalcDistPoints(x1, x2, y1, y2):
    return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

@dataclass
class DetectionConfidence:
    pallet_confidence: float
    pallet_detection: bool
    shelf_confidence: float
    shelf_detection: bool

class Action():
    def __init__(self, TestAction):
        
        # cmd_vel
        self.TestAction = TestAction
        self.cmd_vel = cmd_vel(TestAction)
        # NearbySequence
        self.NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking ')
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.initial_robot_pose_x = 0.0
        self.initial_robot_pose_y = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        self.initial_marker_pose_x = 0.0
        self.initial_marker_pose_y = 0.0
        self.initial_marker_pose_theta = 0.0
        # pallet variable
        self.pallet_2d_pose_x = 0.0
        self.pallet_2d_pose_y = 0.0
        self.pallet_2d_theta = 0.0
        self.pallet_2d_pose_z = 0.0  # 新增的z轴属性
        # Fork_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        self.fork_threshold = 0.005
        # other
        self.check_wait_time = 0
        self.is_triggered = False

        # 初始化 y_pose_history 和窗口大小
        self.y_pose_history = []
        self.moving_average_window = 5
        
        
        self.detectionConfidence = DetectionConfidence(
            pallet_confidence = 0.0,
            pallet_detection = False,
            shelf_confidence = 0.0,
            shelf_detection = False
        )
   

    def SpinOnce(self):
        (self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, 
        self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta,
        self.pallet_2d_pose_x, self.pallet_2d_pose_y, self.pallet_2d_pose_z) = self.TestAction.SpinOnce()   #新增fruits的xyz軸属性

    
    def SpinOnce_fork(self):
        self.updownposition = self.TestAction.SpinOnce_fork()

    def SpinOnce_confidence(self):
        self.detectionConfidence = self.TestAction.SpinOnce_confidence()
    


    def fnseqDeadReckoning(self, dead_reckoning_dist):#(使用里程紀計算)移動到離現在位置dead_reckoning_dist公尺的地方, 1.0 = 朝向marker前進1公尺, -1.0 = 朝向marker後退1公尺
        self.SpinOnce()
        Kp = 0.2
        threshold = 0.015
        if self.is_triggered == False:
            self.is_triggered = True
            self.initial_robot_pose_x = self.robot_2d_pose_x
            self.initial_robot_pose_y = self.robot_2d_pose_y
        dist = math.copysign(1, dead_reckoning_dist) * fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
        if math.copysign(1, dead_reckoning_dist) > 0.0:
            if  (dead_reckoning_dist - dist - threshold) < 0.0:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            else:
                self.cmd_vel.fnGoStraight(Kp, -(dead_reckoning_dist - dist))
                return False
        elif math.copysign(1, dead_reckoning_dist) < 0.0:
            if  dead_reckoning_dist - dist > 0.0:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            else:
                self.cmd_vel.fnGoStraight(Kp, -(dead_reckoning_dist - dist))
                return False


    def fnseqDeadReckoning_fruit(self, dead_reckoning_dist):  # 使用里程計計算後退
        self.SpinOnce()
        Kp = 0.2  # 比例控制器的增益
        threshold = 0.015  # 距離閾值

        if self.is_triggered == False:
            # 初始化觸發狀態，並保存初始位置
            self.is_triggered = True
            self.initial_robot_pose_x = self.robot_2d_pose_x
            self.initial_robot_pose_y = self.robot_2d_pose_y
            # 打印初始化訊息
            self.TestAction.get_logger().info(f"Initial position set at x: {self.initial_robot_pose_x}, y: {self.initial_robot_pose_y}")

        # 計算目前距離與初始位置的差距
        dist = fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)

        # 檢查是否達到目標後退距離
        if (abs(dead_reckoning_dist) - dist - threshold) < 0.0:
            # 如果達到目標距離，停止機器人
            self.cmd_vel.fnStop()
            self.is_triggered = False
            # 打印停止訊息
            self.TestAction.get_logger().info(f"Reached target distance. Stopping. Total distance moved: {dist}")
            return True
        else:
            # 持續後退，`fnGoStraight` 使用負值控制後退
            self.cmd_vel.fnGoStraight(Kp, -(abs(dead_reckoning_dist) - dist))
            # 打印後退訊息
            self.TestAction.get_logger().info(f"Moving backwards. Current distance: {dist}, Remaining distance: {abs(dead_reckoning_dist) - dist}")
            return False



    def fnseqMoveToMarkerDist(self, marker_dist): #(使用marker計算) 移動到距離marker_dist公尺的位置
        self.SpinOnce()
        Kp = 0.2
        if(marker_dist < 2.0):
            threshold = 0.015
        else:
            threshold = 0.03

        dist = math.sqrt(self.marker_2d_pose_x**2 + self.marker_2d_pose_y**2)
        
        if dist < (marker_dist-threshold):
            self.cmd_vel.fnGoStraight(Kp, marker_dist - dist)
            return False
        elif dist > (marker_dist+threshold):
            self.cmd_vel.fnGoStraight(Kp, marker_dist - dist)
            return False
        else:
            self.cmd_vel.fnStop()
            return True
        
    def fnSeqChangingtheta(self, threshod): #旋轉到marker的theta值為0, threshod為角度誤差值
        self.SpinOnce()
        Kp = 0.1
        # self.marker_2d_theta= self.TrustworthyMarker2DTheta(1)
        # print("desired_angle_turn", self.marker_2d_theta)
        # print("threshod", threshod)
        if abs(self.marker_2d_theta) < threshod  :
            self.cmd_vel.fnStop()
            if self.check_wait_time > 5 :
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
                return False
        else:
            self.cmd_vel.fnTurn(Kp, self.Actionmarker_2d_theta)
            self.check_wait_time =0
            return False
        
    def TrustworthyMarker2DTheta(self, duration): #用於計算marker的theta值再duration(取樣時間)中的平均值，避免在抖動的marker的theta值不穩定, 並且去除一個標準差外的極端值
        marker_2d_theta_list = [0.0]
        initial_time = self.TestAction.get_clock().now().nanoseconds / 1e9
        
        while(abs(initial_time - (self.TestAction.get_clock().now().nanoseconds / 1e9)) < duration):
            self.SpinOnce()
            marker_2d_theta_list.append(self.marker_2d_theta)
            # print("self.marker_2d_theta", self.marker_2d_theta)
            time.sleep(0.05)
        # print("marker_2d_theta_list", marker_2d_theta_list)
        threshold = 0.5
        mean = statistics.mean(marker_2d_theta_list)
        stdev = statistics.stdev(marker_2d_theta_list)
        upcutoff = mean + threshold * stdev
        downcutoff = mean - threshold * stdev
        clean_list = []
        for i in marker_2d_theta_list:
            if(i > downcutoff and i < upcutoff):
                clean_list.append(i)
        # print("clean_list", clean_list)
        # print("mean", statistics.mean(clean_list))
        return statistics.median(clean_list) 
    
    
    def fnSeqMovingNearbyParkingLot(self, desired_dist_threshold): #如果desired_dist的值小於desired_dist_threshold, 則不執行此動作
        self.SpinOnce()
        Kp = 0.2
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

                self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(3)
                self.initial_marker_pose_x = self.marker_2d_pose_x
                # print("initial_marker_pose_theta ", self.initial_marker_pose_theta)
                # decide doing fnSeqMovingNearbyParkingLot or not
                desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
                if abs(desired_dist) < desired_dist_threshold:
                    self.is_triggered = False
                    return True
            
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            
            desired_angle_turn = desired_angle_turn
            self.cmd_vel.fnTurn(Kp, desired_angle_turn)

            if abs(desired_angle_turn) < 0.03:
                self.cmd_vel.fnStop()
                if self.check_wait_time >10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            else:
                self.check_wait_time =0    

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

            dist_from_start = fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
            desired_dist = self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
            
            remained_dist = desired_dist + dist_from_start
            self.cmd_vel.fnGoStraight(Kp, desired_dist)

            if abs(remained_dist) < 0.02:
                self.cmd_vel.fnStop()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value
                self.is_triggered = False


        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            # print("desired_angle_turn", desired_angle_turn)
            desired_angle_turn = -1. * desired_angle_turn
            self.cmd_vel.fnTurn(Kp, desired_angle_turn)
            if abs(desired_angle_turn) < 0.01:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            # if abs(desired_angle_turn) < 0.01:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 20:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.initial_turn.value
            #         self.is_triggered = False
            #         return True                
            #     else:
            #         self.check_wait_time =self.check_wait_time  +1
            # elif abs(desired_angle_turn) < 0.02 and self.check_wait_time:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 20:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.initial_turn.value
            #         self.is_triggered = False
            #         return True                
            #     else:
            #         self.check_wait_time =self.check_wait_time  +1
            # else:
            #     self.check_wait_time =0    
        return False
    
    def fnSeqParking(self, parking_dist, kp,object_name):
        self.SpinOnce()
        desired_angle_turn = math.atan2(self.pallet_2d_pose_y - 0, self.pallet_2d_pose_x - 0)
        if self.TFConfidence(object_name):

            if desired_angle_turn <0:
                desired_angle_turn = desired_angle_turn + math.pi
            else:
                desired_angle_turn = desired_angle_turn - math.pi
            self.cmd_vel.fnTrackMarker(desired_angle_turn, kp)
            if (abs(self.pallet_2d_pose_x) < parking_dist)  :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            elif (abs(self.pallet_2d_pose_x) < parking_dist) and self.check_wait_time:
                self.cmd_vel.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            else:
                self.check_wait_time =0
                return False
        else:
            return False
    
    def fnForkFruit(self, z_pose_threshold,object_name):#0~2.7  #透過marker的z軸位置來控制牙叉的上下
        self.SpinOnce_fork()
        self.SpinOnce()

        self.TestAction.get_logger().info(f"Marker 2D Pose: x={self.pallet_2d_pose_x}, y={self.pallet_2d_pose_y}, z={self.pallet_2d_pose_z}")
        if self.TFConfidence(object_name):

            if( self.pallet_2d_pose_z < -z_pose_threshold):
                self.cmd_vel.fnfork(2000.0)
                self.TestAction.get_logger().info("Fork up")
                return False

            elif (self.pallet_2d_pose_z > z_pose_threshold):
                self.cmd_vel.fnfork(-2000.0)
                self.TestAction.get_logger().info("Fork down")
                return False
            else :
                self.cmd_vel.fnfork(0.0)
                self.TestAction.get_logger().info("Fork stop")
                return True
        else:
            return False
               
    def fnForkFruit_approach(self, x_pose_threshold,object_name):#0~2.7  #透過marker的x軸位置來控制叉車前進
        self.SpinOnce_fork()
        self.SpinOnce()

        self.TestAction.get_logger().info(f"Marker 2D Pose: x={self.pallet_2d_pose_x}, y={self.pallet_2d_pose_y}, z={self.pallet_2d_pose_z}")
        if self.TFConfidence(object_name):
        
            if( self.pallet_2d_pose_x < x_pose_threshold):
                self.cmd_vel.fnGoStraight_fruit()
                self.TestAction.get_logger().info("GoStraight")
                return False
            else:
                self.cmd_vel.fnStop()
                self.TestAction.get_logger().info("Stop")
                return True
        else:
            return False
        

    def fnForkFruit_approach_y(self, y_pose_threshold_min, y_pose_threshold_max, object_name):
        """
        基於 y 軸進行粗略逼近，將車輛移動到設定範圍內。
        """
        self.SpinOnce_fork()
        self.SpinOnce()

        smoothed_y = self.compute_moving_average(self.pallet_2d_pose_y)
        self.TestAction.get_logger().info(
            f"Marker 2D Pose (Smoothed): y={smoothed_y}"
        )

        if self.TFConfidence(object_name):
            if y_pose_threshold_min <= smoothed_y <= y_pose_threshold_max:
                self.cmd_vel.fnStop()
                self.TestAction.get_logger().info("Within Threshold - Stopping")
                return True
            elif smoothed_y > y_pose_threshold_max:
                self.cmd_vel.fnGoBack()
                self.TestAction.get_logger().info("Moving Backward")
                return False
            elif smoothed_y < y_pose_threshold_min:
                self.cmd_vel.fnGoStraight_fruit()
                self.TestAction.get_logger().info("Moving Forward")
                return False
        else:
            self.cmd_vel.fnStop()
            self.TestAction.get_logger().warn("TF Data Not Confident - Stopping")
            return False


    def stabilize_and_check_y(self, object_name, stabilization_time=3.0):
        """
        停止後等待 y 軸數據穩定，並返回穩定的 y 值。
        """
        stable_y_values = []
        start_time = time.time()

        while time.time() - start_time < stabilization_time:
            self.SpinOnce()
            smoothed_y = self.compute_moving_average(self.pallet_2d_pose_y)
            stable_y_values.append(smoothed_y)
            time.sleep(0.1)  # 每 100ms 更新一次

        final_y = sum(stable_y_values) / len(stable_y_values)
        self.TestAction.get_logger().info(f"Stable Y Value: {final_y}")
        return final_y


    def refine_alignment(self, object_name, target_y, max_iterations=30, threshold=0.005):
        """
        當水果位於相機的左/右（以 y 軸衡量）時，對底盤做小幅微調。
        同時在每次迭代中檢查 TFConfidence(object_name)，若不可信就停止並返回。

        :param object_name: 給 TFConfidence() 判斷的標的名稱
        :param target_y: 目標的 y 軸位置
        :param max_iterations: 最大迭代次數
        :param threshold: (這裡僅保留參數，但實際不使用判斷)
        :return: bool, True=完成對準，False=超過最大次數或不符合 TFConfidence
        """

        Y_MIN = -0.050
        Y_MAX = -0.040

        for i in range(max_iterations):
            # 在每次開始前先更新資料
            self.SpinOnce_fork()
            self.SpinOnce()

            # -- 檢查 TFConfidence(object_name) --
            if not self.TFConfidence(object_name):
                self.cmd_vel.fnStop()
                self.TestAction.get_logger().warn(
                    f"TF Data Not Confident for object '{object_name}' - Stopping"
                )
                return False

            # 1) 讀取當前 Y (平滑處理)
            smoothed_y = self.compute_moving_average(self.pallet_2d_pose_y)
            error = smoothed_y - target_y
            self.TestAction.get_logger().info(
                f"[refine_alignment] Iter {i+1}, Camera Y = {smoothed_y:.3f}, Error = {error:.3f}"
            )

            # === [原本] 2) 若誤差已小於閾值 => 視為對準完成 ===
            # 已移除(或註解掉)以下程式碼，以免在 -0.036 等狀況就提早 return True
            #
            # if abs(error) < threshold:
            #     self.cmd_vel.fnStop()
            #     self.TestAction.get_logger().info("Alignment Complete by threshold")
            #     return True

            # 3) 根據偏差方向決定往前或往後移動
            if error > 0:
                self.cmd_vel.fnGoBack()
                self.TestAction.get_logger().info("Moving Backward to Align")
            else:
                self.cmd_vel.fnGoStraight_fruit()
                self.TestAction.get_logger().info("Moving Forward to Align")

            # 4) 移動 0.2 秒 (可自行調整)
            time.sleep(0.2)

            # 5) 停止，等待 3 秒做穩定
            self.cmd_vel.fnStop()
            self.TestAction.get_logger().info("Stop, waiting 3s to re-check y error...")
            time.sleep(3)

            # -- 再次檢查 TFConfidence(object_name) (可選) --
            if not self.TFConfidence(object_name):
                self.cmd_vel.fnStop()
                self.TestAction.get_logger().warn(
                    f"TF Data Not Confident for object '{object_name}' after waiting - Stopping"
                )
                return False

            # 6) 再讀取 y
            self.SpinOnce_fork()
            self.SpinOnce()
            smoothed_y = self.compute_moving_average(self.pallet_2d_pose_y)
            error = smoothed_y - target_y
            self.TestAction.get_logger().info(
                f"After waiting: Camera Y = {smoothed_y:.3f}, Error = {error:.3f}"
            )

            # === [原本] 7) 再次檢查誤差閾值 ===
            # 一樣移除或註解，以免提早 return True
            #
            # if abs(error) < threshold:
            #     self.cmd_vel.fnStop()
            #     self.TestAction.get_logger().info("Alignment Complete (After waiting by threshold)")
            #     return True

            # 8) 檢查是否落入 [-0.050, -0.040] 區間
            if Y_MIN <= smoothed_y <= Y_MAX:
                self.TestAction.get_logger().info(
                    f"Current Y={smoothed_y:.3f} in range [{Y_MIN}, {Y_MAX}], checking 10-sample average..."
                )
                stable_y_vals = []
                for _ in range(10):
                    self.SpinOnce_fork()
                    self.SpinOnce()

                    if not self.TFConfidence(object_name):
                        self.cmd_vel.fnStop()
                        self.TestAction.get_logger().warn(
                            f"TF Data Not Confident for object '{object_name}' in 10-sample check - Stopping"
                        )
                        return False

                    val = self.compute_moving_average(self.pallet_2d_pose_y)
                    stable_y_vals.append(val)
                    time.sleep(0.1)

                final_avg = sum(stable_y_vals) / len(stable_y_vals)
                self.TestAction.get_logger().info(f"10-sample average Y: {final_avg:.3f}")

                if Y_MIN <= final_avg <= Y_MAX:
                    self.cmd_vel.fnStop()
                    self.TestAction.get_logger().info(
                        f"Y is stable in [{Y_MIN}, {Y_MAX}], alignment complete!"
                    )
                    return True
                else:
                    self.TestAction.get_logger().info(
                        f"10-sample average Y not in range [{Y_MIN}, {Y_MAX}] => continue adjusting"
                    )

        # 超過最大迭代次數
        self.cmd_vel.fnStop()
        self.TestAction.get_logger().warn("Failed to Align Within Max Iterations")
        return False





        
    def fnSeqdecide(self, decide_dist):#decide_dist偏離多少公分要後退
        self.SpinOnce()
        dist = self.marker_2d_pose_y
        if  abs(dist) < abs(decide_dist):
            return True
        else:
            return False

    def fnForkUpdown(self, desired_updownposition):#0~2.7
        self.SpinOnce_fork()
        fork_threshold = 0.001
        if(desired_updownposition < 0):
            return True

        if self.updownposition < desired_updownposition - fork_threshold:
            self.cmd_vel.fnfork(2000.0)
            return False
        dist = self.marker_2d_pose_y
        if self.updownposition > desired_updownposition + fork_threshold:
            self.cmd_vel.fnfork(-2000.0)
            return False
        else:
            self.cmd_vel.fnfork(0.0)
            return True

    def TFConfidence(self, object_name):#判斷TF是否可信
        self.SpinOnce_confidence()

        if object_name == "bodycamera":
            if (not self.detectionConfidence.pallet_detection) or self.detectionConfidence.pallet_confidence < self.TestAction.confidence_minimum:
                self.cmd_vel.fnStop()
                return False
        return True
    
    def is_y_stable(self):
        """
        檢查 y 值是否穩定。使用最近的數值判斷波動是否在允許範圍內。
        """
        if len(self.y_pose_history) < self.moving_average_window:
            return False  # 如果數據不足，視為不穩定
        recent_values = self.y_pose_history[-self.moving_average_window:]
        max_value = max(recent_values)
        min_value = min(recent_values)
        return (max_value - min_value) < 0.005  # 假設允許的波動範圍為 0.005

    def compute_moving_average(self, new_value):
        """
        計算滑動平均值。
        """
        # 將新數值加入歷史紀錄
        self.y_pose_history.append(new_value)

        # 若歷史數據超過窗口大小，移除最舊數據
        if len(self.y_pose_history) > self.moving_average_window:
            self.y_pose_history.pop(0)

        # 計算平均值
        return sum(self.y_pose_history) / len(self.y_pose_history)

class cmd_vel():
    def __init__(self, TestAction):
        self.pub_cmd_vel = TestAction.cmd_vel_pub
        self.pub_fork = TestAction.fork_pub

    def cmd_pub(self, twist): #限制速度的範圍
        if twist.linear.x > 0.15:
            twist.linear.x =0.05
        elif twist.linear.x < -0.2:
            twist.linear.x =-0.05
        if twist.linear.x > 0 and twist.linear.x < 0.02:
            twist.linear.x =0.02
        elif twist.linear.x < 0 and twist.linear.x > -0.02:
            twist.linear.x =-0.02

        if twist.angular.z > 0.2:
            twist.angular.z =0.1
        elif twist.angular.z < -0.2:
            twist.angular.z =-0.2                     
        if twist.angular.z > 0 and twist.angular.z < 0.05:
            twist.angular.z =0.03
        elif twist.angular.z < 0 and twist.angular.z > -0.05:
            twist.angular.z =-0.03
        
        self.pub_cmd_vel.publish(twist)

    def fnStop(self):
        twist = Twist()
        self.cmd_pub(twist)

    def fnTurn(self, Kp=0.2, theta=0.):
        twist = Twist()
        twist.angular.z = Kp * theta
        self.cmd_pub(twist)

    def fnGoStraight(self, Kp=0.2, v=0.):
        twist = Twist()
        twist.linear.x = Kp * v
        self.cmd_pub(twist)

    def fnfork(self,velocity):
        fork_msg = Meteorcar()
        fork_msg.fork_velocity = velocity
        self.pub_fork.publish(fork_msg)

    def fnTrackMarker(self, theta, Kp):
        # Kp = 2.0 #6.5
        twist = Twist()
        twist.linear.x = -0.025
        twist.angular.z = Kp * theta
        self.cmd_pub(twist)

        
    def SpinOnce_fork(self):
        rclpy.spin_once(self)
        return self.updownposition

    def fnGoStraight_fruit(self):      #控制叉車前進
        twist = Twist()
        twist.linear.x = 0.05
        self.cmd_pub(twist)

    def fnGoBack(self):      #控制叉車前進
        twist = Twist()
        twist.linear.x = -0.05
        self.cmd_pub(twist)

def main(args=None):
    rclpy.init(args=args)

    test_action = TestAction()
    try:
        rclpy.spin(test_action)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

'''    def SpinOnce_fork(self):
        rclpy.spin_once(self)
        return self.updownposition
    
def main(args=None):
    rclpy.init(args=args)

    test_action = TestAction()
    try:
        rclpy.spin(test_action)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

'''