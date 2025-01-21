#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from enum import Enum
from action import Action
import time
import rclpy
ParkingBodyCameraSequence = Enum( 'ParkingBodyCameraSequence', \
                        'init_fork \
                        changing_direction \
                        move_nearby_parking_lot \
                        parking \
                        changingtheta \
                        decide \
                        back \
                        stop \
                        error')
ParkingForkCameraSequence = Enum( 'ParkingForkCameraSequence', \
                        'init_fork \
                        changing_direction \
                        parking \
                        changingtheta \
                        decide \
                        back \
                        stop \
                        error')
RaisePalletSequence = Enum( 'RaisePalletSequence', \
                        'init_fork \
                        dead_reckoning \
                        fork_updown \
                        back \
                        stop \
                        error')
DropPalletSequence = Enum( 'DropPalletSequence', \
                        'init_fork \
                        dead_reckoning \
                        fork_updown \
                        back \
                        stop \
                        error')
FruitSequence = Enum( 'FruitSequence', \
                        'fork_updown \
                        dead_reckoning \
                        changing_direction \
                        dead_reckoning_x \
                        y_axis_fineturn \
                        move_forward \
                        end \
                        parking \
                        back \
                        stop \
                        error')
class ActionSequence():
    def __init__(self, VisualServoingActionServer):
        self.visual_servoing_action_server = VisualServoingActionServer  # access function {SpinOnce(), SpinOnce_Fork()} and parameter and Node{ROS2 API}
        self.action = Action(VisualServoingActionServer)
        self.is_sequence_finished = False

    def parking_bodycamera(self, goal_handle, layer):
        current_sequence = ParkingBodyCameraSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            feedback = str(ParkingBodyCameraSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == ParkingBodyCameraSequence.init_fork.value):
                self.is_sequence_finished = self.action.fnForkUpdown()
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.changing_direction.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingBodyCameraSequence.changing_direction.value):
                self.is_sequence_finished = self.action.fnSeqChangingDirection(self.visual_servoing_action_server.bodycamera_ChangingDirection_threshold)

                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.move_nearby_parking_lot.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingBodyCameraSequence.move_nearby_parking_lot.value):
                self.is_sequence_finished = self.action.fnSeqMovingNearbyParkingLot(self.visual_servoing_action_server.bodycamera_desired_dist_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.parking.value):
                self.is_sequence_finished = self.action.fnSeqParking(self.visual_servoing_action_server.bodycamera_parking_stop, 2.0)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.changingtheta.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.changingtheta.value):
                self.is_sequence_finished = self.action.fnSeqChangingtheta(self.visual_servoing_action_server.bodycamera_Changingtheta_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.decide.value):
                self.is_sequence_finished = self.action.fnSeqdecide(self.visual_servoing_action_server.bodycamera_decide_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.stop.value
                    self.is_sequence_finished = False
                elif self.is_sequence_finished == False:
                    current_sequence = ParkingBodyCameraSequence.back.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.back.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.bodycamera_back_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingBodyCameraSequence.stop.value):
                return
            
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return
            
    def parking_forkcamera(self, goal_handle, layer):
        current_sequence = ParkingForkCameraSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            feedback = str(ParkingForkCameraSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == ParkingForkCameraSequence.init_fork.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.forkcamera_parking_fork_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.forkcamera_parking_fork_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingForkCameraSequence.parking.value):
                self.is_sequence_finished = self.action.fnSeqParking(self.visual_servoing_action_server.forkcamera_parking_stop, 1.0)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.changingtheta.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.changingtheta.value):
                self.is_sequence_finished = self.action.fnSeqChangingtheta(self.visual_servoing_action_server.forkcamera_Changingtheta_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.decide.value):
                self.is_sequence_finished = self.action.fnSeqdecide(self.visual_servoing_action_server.forkcamera_decide_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.stop.value
                    self.is_sequence_finished = False
                elif self.is_sequence_finished == False:
                    current_sequence = ParkingForkCameraSequence.back.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.back.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.forkcamera_back_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingForkCameraSequence.stop.value):
                return
            
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return

    def raise_pallet(self, goal_handle, layer):
        current_sequence = RaisePalletSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            feedback = str(RaisePalletSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == RaisePalletSequence.init_fork.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_fork_init_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_fork_init_layer1)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.dead_reckoning.value
                    feedback = str(RaisePalletSequence(current_sequence))
                    self.visual_servoing_action_server.get_logger().info('fnseqDeadReckoning change to:{0}'.format(feedback))
                    self.is_sequence_finished = False

                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.dead_reckoning.value
                    feedback = str(RaisePalletSequence(current_sequence))
                    self.visual_servoing_action_server.get_logger().info('fnseqDeadReckoning change to:{0}'.format(feedback))
                    self.is_sequence_finished = False

                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.fork_updown.value
                    self.is_sequence_finished = False

                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.fork_updown.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_raise_height_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_raise_height_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.back.value
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.back.value):
                self.is_sequence_finished = self.action.fnseqDeadReckoning(self.visual_servoing_action_server.raise_pallet_back_dist)

                if self.is_sequence_finished == True:
                    return
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return
              
    def drop_pallet(self, goal_handle, layer):
        current_sequence = DropPalletSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)

            if(current_sequence == DropPalletSequence.init_fork.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_fork_init_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_fork_init_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.dead_reckoning.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.dead_reckoning.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.drop_pallet_dead_reckoning_dist)
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.fork_updown.value):
                if layer == 1:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoingforward_distance_action_server.drop_pallet_drop_height_layer1)
                elif layer == 2:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_drop_height_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.back.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.back.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.drop_pallet_back_distance)

                if self.is_sequence_finished == True:
                    return
            else:   
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return
    def fruit_docking(self, goal_handle):  #製作一個用於水果對接的函數
        current_sequence = FruitSequence.dead_reckoning_x.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)



            if current_sequence == FruitSequence.dead_reckoning_x.value:
                # 使用 dead reckoning 基於 y 軸距離移動至目標範圍
                self.is_sequence_finished = self.action.fnForkFruit_approach_y(
                    y_pose_threshold_min=-0.06,
                    y_pose_threshold_max=-0.04,
                    object_name="bodycamera"  # 使用的 marker 名稱
                )

                if self.is_sequence_finished:
                    current_sequence = FruitSequence.y_axis_fineturn.value  # 切換至下一狀態
                    self.is_sequence_finished = False  # 重置狀態標誌

            elif current_sequence == FruitSequence.y_axis_fineturn.value:
                # 使用 dead reckoning 基於 y 軸距離移動至目標範圍
                self.is_sequence_finished = self.action.stabilize_and_check_y(
                    object_name="bodycamera"  # 使用的 marker 名稱
                )

                if self.is_sequence_finished:
                    current_sequence = FruitSequence.move_forward.value  # 切換至下一狀態
                    self.is_sequence_finished = False  # 重置狀態標誌

            elif current_sequence == FruitSequence.move_forward.value:
                # 關鍵：要接收 refine_alignment() 的回傳值
                self.is_sequence_finished = self.action.refine_alignment(
                    object_name="bodycamera",
                    target_y=-0.045,
                    max_iterations=10,
                    threshold=0.005
                )

                # 如果對準完成 (True) => 結束流程，或切到下一個狀態
                if self.is_sequence_finished:
                    self.visual_servoing_action_server.get_logger().info("Process completed successfully.")
                    # 結束整個函式
                    return

                # 如果不想直接結束，也可以切到下一個狀態 (範例)
                # if self.is_sequence_finished:
                #     current_sequence = FruitSequence.pick_fruit.value
                #     self.is_sequence_finished = False

            else:
                self.visual_servoing_action_server.get_logger().info(
                    f'Error: {current_sequence} does not exist'
                )
                return


