#!/usr/bin/env python
# TODO: write the patrol FSM action


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

from typing import List
from merlin2_hospital_patrolling.pddl import room_type, room_at, room_patrolled
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at
from merlin2_fsm_action import Merlin2FsmAction
from merlin2_fsm_action import Merlin2BasicStates

from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin import CbState

from kant_dto import PddlObjectDto, PddlConditionEffectDto




class Merlin2RoomPatrolFsnAction(Merlin2FsmAction):

    def __init__(self) -> None:

        self._room = PddlObjectDto(room_type, "room")
        self._wp = PddlObjectDto(wp_type, "wp")

        super().__init__("room_patrol")

        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state(
            "ROTATING",
            CbState([SUCCEED], self.rotate),
            transitions={
                SUCCEED: "PREPARING_TEXT"
            }
        )

        self.add_state(
            "PREPARING_TEXT",
            CbState([SUCCEED], self.prepare_text),
            transitions={
                SUCCEED: "SPEAKING"
            }
        )

        self.add_state(
            "SPEAKING",
            tts_state
        )


        # Rotacion #


        
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)


    def rotate(self, blackboard: Blackboard) -> str:

        msg = Twist()
        
        
        msg.angular.z = 0.1
        self.publisher_.publish(msg)

        time.sleep(10)
    
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Rotación completa')

        

        return SUCCEED

    def prepare_text(self, blackboard: Blackboard) -> str:
        blackboard.text = "Strecher room patrolled"
        return SUCCEED

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self._room, self._wp]
    
    def create_conditions(self) -> List[PddlConditionEffectDto]:

        cond_2 = PddlConditionEffectDto(
            robot_at,
            [self._wp],
            PddlConditionEffectDto.AT_START
        )

        cond_3 = PddlConditionEffectDto(
            room_at,
            [self._room, self._wp],
            PddlConditionEffectDto.AT_START
        )

        return [cond_2, cond_3]
    
    def create_efects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]
    
def main():
    rclpy.init()
    node = Merlin2RoomPatrolFsnAction()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()