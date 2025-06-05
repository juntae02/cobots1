# project name : 나의 완벽한 비서 (장인 보조 협동 로봇 - 대장간 프로젝트)
# date : 2025-06-05
# author : 두산 로벨스
# ver : 1.0 찐찐 최종
'''
TCPServer 클래스에서 모든 동작들을 시키는데  init에 선언된 클래스들을 확인 하면 어떤 동작을 수행 시키는가 시퀀스를 이해하기 쉬울 것임.
    def __init__(self, host='0.0.0.0', port=1234):
        self.sp = SpitRotationTask() 단조 - 비틀림
        self.qc = QCManager() QC무게 측정
        self.mcup = MoveCup() 주조
        self.fg = Forge() 단조 - 수직, 해머, 잡기
        self.gt = GrindingTask() - 단조 수평
        self.mj=Move_Js() - j 움직임 예외 처리
참고로 print문은 자동으로 휴대폰 tcp연결된 곳으로 가며, 한글 지원 안한다. 그걸 pc에서 확인 하고 싶으면 client_print_redirector 3개 있는거 전부 주석 처리하면됨.
-> 폰으로 확인하면 에러 상황에 휴대폰 연결이 끊기며 정지함. 그래서 무슨 에러인지 확인은 어려움

'''

import rclpy
import DR_init
import threading
from datetime import datetime
import time
import numpy as np
import os
from copy import deepcopy
import json
import csv
import socket
import subprocess
import math
import sys
from io import StringIO
from DR_common2 import posx, posj

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init(args=None)
node = rclpy.create_node("qc_management", namespace=ROBOT_ID)

DR_init.__dsr__node = node


from DSR_ROBOT2 import (
    movel,
    movel,
    amovej, 
    movesj, 
    movesx, 
    move_periodic,
    amove_periodic,
    reset_workpiece_weight,
    task_compliance_ctrl, 
    set_desired_force, 
    check_force_condition, 
    release_compliance_ctrl, 
    release_force,
    set_tool, 
    set_tcp, 
    get_current_posx, 
    wait,
    get_workpiece_weight,
    get_tool_force,
    set_digital_output,
    get_digital_input,
    ikin,
    check_motion,
    DR_BASE, 
    DR_TOOL, 
    DR_AXIS_X, 
    DR_AXIS_Y, 
    DR_AXIS_Z, 
    DR_FC_MOD_REL,
    OFF,
    ON,
)
# 위치 도달 확인용 허용 오차
POS_TOLERANCE = 10.0  # mm 단위

POS_J = '1'
POS_X = '0'

GRIP = 1
RELEASE = 2

DR_ACC_L = 70
DR_VEL_L = 70
DR_ACC_J = 70
DR_VEL_J = 70

VELOCITY, ACC = 100, 100

TARGET_FORCE = 10

CHECK_FORCE = 2.4

COMPLIANCE_AXIS = 2000
COMPLIANCE = 20

FORCE_THRESHOLD = 20.0 
# 전역변수
wait_poseJ = [1.78,38.87,88.84,-179.96,127.93,-93.51]
moru_J=[4.25,34.93,57.56,0.22,87.55,4.31]

holding1J=[33.59,53.62,28.86,-36.83,114.41,20.42]
holding2J=[29.45,51.93,25.10,-25.24,112.71,20.01]
holding3J=[26.12,50.26,23.14,-16.19,112.55,18.50]
holding4J=[22.24,49.32,22.25,-8.86,111.96,18.50]
## 0. 집으로 이동
JReady = [0, 0, 90, 0, 90, 0]
home = posj(JReady)

## 1. 컵 위치까지 이동
deg1 = [2.68, 46.84, 75.27, 90.16, -90.72, 35.87]   # 도 단위
p1_1 = posj(deg1)   # 컵 있는 위치로 이동
p1_2 = posx([623.690, -424.150, 226.040, 91.71, -89.53, -86.24])   # 가까이 가기 위해 앞으로 movel
# 424.150에서 찍고, 497.910까지

# 4. 물 붓는 위치
pour_position = posx([420.990, -155.200, 198.160, 91.71, -89.53, -86.24])  

# 5. 컵 내려 놓기
    ### 라디안 각도가 필요하지 않을 시, 여기 부분 변경(2|4)
deg2 = [16.86, 49.07, 60.98, 94.97, -104.06, 24.34]   # 도 단위
# rad2 = [radians(d2) for d2 in deg2]                 # 라디안 단위
p5_1 = posj(deg2)   # 경유점1

deg3 = [-12.66, 47.76, 68.60, 83.15, -77.03, 31.00]   # 도 단위
# rad3 = [radians(d3) for d3 in deg3]                 # 라디안 단위
p5_2 = posj(deg3)   # 경유점2

##### x 마이너스로 1.2, y 플러스로 2.0
deg4 = [-11.6, 48.95, 73.14, 80.8, -79.49, 37.23]   # 도 단위
# rad4 = [radians(d4) for d4 in deg4]                 # 라디안 단위
p5_3 = posj(deg4)   # 경유점3

# p5_4 = posx([620.050, -208.790, 223.110, 91.71, -89.53, -86.24]) 

set_tool("TCP208mm")
set_tcp("GripperSA_rg2_250509")

# stdout을 TCP 클라이언트로 보내는 클래스

class ClientPrintRedirector:
    def __init__(self):
        self.clients = []  # 연결된 클라이언트 소켓 리스트
        self.buffer = StringIO()  # 출력 버퍼

    def add_client(self, client_socket):
        self.clients.append(client_socket)
        self.write(f"ClientPrintRedirector: Added client {client_socket.getpeername()}")

    def remove_client(self, client_socket):
        if client_socket in self.clients:
            self.clients.remove(client_socket)
            self.write(f"ClientPrintRedirector: Removed client {client_socket.getpeername()}")

    def write(self, message):
        if message.strip():  # 빈 문자열 제외
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            formatted_message = f"[LOG] {timestamp} - {message.strip()}\n"
            for client in self.clients[:]:  # 복사본으로 순회
                try:
                    client.send(formatted_message.encode('utf-8'))
                except:
                    self.clients.remove(client)  # 연결 끊긴 클라이언트 제거
        self.buffer.write(message)

    def flush(self):
        pass

# stdout을 ClientPrintRedirector로 리다이렉트
client_print_redirector = ClientPrintRedirector()
sys.stdout = client_print_redirector

class Move_Js:
    def __init__(self):
        pass

    def move_j(self,pos,vel=10,acc=10):
        flag=False
        while True:
            print("go_amove")
            amovej(pos, vel=vel, acc=acc)
            time.sleep(1.0)
            if check_motion()==0:   # 모션이 완료 된 경우
                print("최종 목표 위치 도달")
                break
            while True:
                fx, fy, fz, tx, ty, tz = get_tool_force()
                # fx, fy, fz, tx, ty, tz = fake_get_tool_force()
                total_force = (fx**2 + fy**2 + fz**2)**0.5

                if total_force > FORCE_THRESHOLD:
                    print(f"⚠ 외력 감지됨: {total_force:.2f} N → 모션 정지")
                    #MoveStop
                    # 순응 제어 활성화 (XYZ 방향만 허용)
                    task_compliance_ctrl([1, 1, 1, 0, 0, 0])
                    time.sleep(0.5)
                    release_compliance_ctrl()
                    time.sleep(0.5)
                    break
                if check_motion()==0:   # 모션이 완료 된 경우
                    flag=True
                    break
            if flag:
                print("최종 목표 위치 도달")
                break

# ────── 그리퍼 동작 ──────
def release():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)

def grip():
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)


# ────── 컵 이동 클래스 정의 ──────
class MoveCup:
    def __init__(self):
        self.repeat_m = 5
        self.mj=Move_Js()

    def grip(self):
        set_digital_output(RELEASE, OFF)

    def grip_all(self):
        set_digital_output(RELEASE, OFF)
        set_digital_output(GRIP, OFF)

    def release(self):
        set_digital_output(GRIP, ON)
        set_digital_output(RELEASE, ON)
        wait(1.0)

    def move_cup_position(self):
        print("주조 공정 시작")  # 추가
        print("레들 위치로 이동 중...")  # 추가
        self.mj.move_j(p1_1, vel=DR_VEL_J, acc=DR_ACC_J)
        movel(p1_2, vel=DR_VEL_L, acc=DR_ACC_L)
        
        # 순응제어 + (-)y방향 힘
        task_compliance_ctrl(stx=[20000, 200, 20000, 20, 20, 20])
        time.sleep(0.1)
        set_desired_force(fd=[0, -5, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], time=0.0, mod=DR_FC_MOD_REL)

        # 닿으면 좌표 받기
        while True:
            if check_force_condition(axis=DR_AXIS_Z, max=2.5, ref=DR_TOOL):
                release_force()
                release_compliance_ctrl()
                wait(0.5)
                self.cup_position = get_current_posx()[0]
                print("레들 감지")  # 추가
                print(self.cup_position)    # [623.517, -437.059, 225.996, 91.883, -89.140, -86.459]    497.910까지
                break

        # self.cup_position = get_current_posx()[0]
        # print(self.cup_position)    # [620.050, -418.000, 223.110, 91.710, -89.530, -86.240]


    def grip_cup(self):
        self.target_posx1 = deepcopy(self.cup_position)
        self.target_posx1[1] += 70.0
        movel(posx(self.target_posx1), vel=DR_VEL_L, acc=DR_ACC_L, ref=DR_BASE)
        print("release")  # 추가
        self.release()
        self.target_posx1[1] -= 126.0
        movel(posx(self.target_posx1), vel=DR_VEL_L, acc=DR_ACC_L, ref=DR_BASE)
        print("grip")  # 추가
        self.grip()

    def move_goal_position(self):
        print("거푸집 위치로 이동 시작")  # 추가
        self.target_posx2 = deepcopy(self.target_posx1)
        self.target_posx2[2] += 31.0
        print("거푸집 위치로 이동 중...")  # 추가
        movel(posx(self.target_posx2), vel=DR_VEL_L, acc=DR_ACC_L, ref=DR_BASE)

        self.target_posx2[0] += 64.28
        self.target_posx2[1] += 367.26
        x1 = posx(self.target_posx2)
        self.target_posx2[0] -= 164.03
        self.target_posx2[1] += 249.57
        x2 = posx(self.target_posx2)
        self.target_posx2[0] -= 99.31
        self.target_posx2[1] -= 167.14
        x3 = posx(self.target_posx2)
        print("거푸집 위치로 천천히 이동 중...")  # 추가
        movesx([x1, x2, x3], vel=[50, 20], acc=[60, 30])
        print("거푸집 위치로 이동 완료")  # 추가

    def pour_action(self):
        ##
        print("용탕 균일화 시작")  # 추가
        move_periodic(
            amp=[15.0, 20.0, 0.0, 0.0, 0.0, 0.0],
            period=[2.4, 1.8, 0.0, 0.0, 0.0, 0.0],
            atime=0.2,
            # repeat=5,
            repeat=self.repeat_m,
            ref=DR_BASE
        )
        print("용탕 균일화 완료")  # 추가

        print("용탕 주입 시작")  # 추가
        movel(pour_position, vel=DR_VEL_L, acc=DR_ACC_L)
        self.pour_pose = deepcopy(pour_position)
        self.pour_pose[3:] = [94.42, -157.26, -82.15]
        print("용탕 주입 중, 1/2")  # 추가
        movel(posx(self.pour_pose), vel=24, acc=20)
        wait(0.5)
        self.pour_pose[3:] = [82.71, 166.37, -93.31]
        movel(posx(self.pour_pose), vel=24, acc=20)
        print("용탕 주입 중, 2/2")  # 추가
        movel(pour_position, vel=DR_VEL_L, acc=DR_ACC_L)
        print("용탕 주입 완료")  # 추가

    def final_action(self):
        print("레들 원위치 시작")  # 추가
        movesj([p5_1, p5_2, p5_3], vel=DR_VEL_J, acc=DR_ACC_J)
        self.release()
        print("레들 원위치 완료")  # 추가
        wait(0.2)
        self.final_position = get_current_posx()[0]
        self.final_position[1] += 100.0
        movel(self.final_position, vel=DR_VEL_L, acc=DR_ACC_L)
        self.grip_all()
        print("주조 공정 완료")  # 추가

    def run(self,move_repeat=5):
        self.release()
        time.sleep(1.0)
        self.grip()
        time.sleep(1.0)
        self.repeat_m=move_repeat
        self.mj.move_j(home, vel=DR_VEL_J, acc=DR_ACC_J)
        self.move_cup_position()
        wait(1)
        self.grip_cup()
        wait(1)
        self.move_goal_position()
        wait(1)
        self.pour_action()
        wait(1)
        self.final_action()


# ────── GrindingTask 클래스 ──────
class GrindingTask:
    def __init__(self):
        self.J_READY = [0, 0, 90, 0, 90, 0]
        self.wait_poseJ = [1.78, 38.87, 88.84, -179.96, 127.93, -93.51]
        self.PRE_ANVIL_CONTACT = posx([820.91, -297.59, 503.38, 123.22, -90.2, 28.28])
        self.AFTER_ANVIL_CONTACT = posx([926.79, -56.24, 306.90, 159.42, -115.64, 62.35])
        self.ANVIL_CONTACT = posx([671.92, -358.47, 433.59, 95.53, -92.94, 0.53])
        self.mj=Move_Js()

        self.Z_FORCE = -60.0
        self.AMP_Y = 25.0
        self.PERIOD_Y = 0.7
        self.ACC_TIME = 0.3
        self.SLEEP_MARGIN = 1.0
        self.update_repeat(15)

    def update_repeat(self, repeat_cnt):
        self.repeat_cnt = repeat_cnt
        self.repeat_time = self.PERIOD_Y * self.repeat_cnt

    def set_tool_and_tcp(self):
        set_tool("Tool Weight_3_24")
        set_tcp("TCP208mm")

    def move_to_initial_pose(self):
        print("[INFO] 초기 자세로 이동")
        self.mj.move_j(self.J_READY, vel=60, acc=ACC)

    def move_to_initial_grip_pose(self):
        print("[INFO] 초기 수령 자세로 이동")
        self.mj.move_j(self.wait_poseJ, vel=60, acc=ACC)

    def wait_tool(self):
        release()
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while True:
            if (
                check_force_condition(axis=DR_AXIS_Z, max=20, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y, max=20, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X, max=20, ref=DR_TOOL)
            ):
                print('터치 감지 되었습니다.')
                time.sleep(3)
                grip()
                print('잡았습니다.')
                time.sleep(0.5)
                break

    def release_tool(self):
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while True:
            if (
                check_force_condition(axis=DR_AXIS_Z, max=20, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y, max=20, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X, max=20, ref=DR_TOOL)
            ):
                print('터치 감지 되었습니다.')
                time.sleep(3)
                release()
                print('release 성공.')
                time.sleep(0.5)
                break

    def move_to_anvil_contact(self):
        print("[INFO] 모루 접촉 지점으로 이동")
        movel(self.AFTER_ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(self.PRE_ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(self.ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    def apply_compliance_and_force(self):
        print("[INFO] 순응 제어 및 Z축 힘 제어 적용")
        task_compliance_ctrl([2000, 2000, 200, 300, 300, 300])
        set_desired_force(
            [0, 0, self.Z_FORCE, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            DR_FC_MOD_REL
        )

    def perform_periodic_motion(self):
        time.sleep(2.0)
        print("[INFO] amove_periodic 시작 (Y축 진동)")
        amove_periodic(
            amp=[0.0, self.AMP_Y, 0.0, 0.0, 0.0, 0.0],
            period=[0.0, self.PERIOD_Y, 0.0, 0.0, 0.0, 0.0],
            atime=self.ACC_TIME,
            repeat=self.repeat_cnt,
            ref=DR_BASE
        )
        print("[DEBUG] 진동 시간 대기")
        time.sleep(self.repeat_time + self.SLEEP_MARGIN)

    def shutdown_and_return(self):
        print("[INFO] 진동 종료 → 정지 위치 복귀")
        release_compliance_ctrl()
        movel(self.ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(self.PRE_ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(self.AFTER_ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        self.mj.move_j(self.J_READY, vel=60, acc=ACC)

    def grinding_run(self,num=15):
        self.update_repeat(num)
        self.set_tool_and_tcp()
        self.move_to_initial_pose()
        self.move_to_initial_grip_pose()
        self.wait_tool()
        self.move_to_initial_pose()
        self.move_to_anvil_contact()
        self.apply_compliance_and_force()
        self.perform_periodic_motion()
        self.shutdown_and_return()
        self.move_to_initial_grip_pose()
        self.release_tool()


# ────── 비틀기 전용 클래스 ──────
class SpitRotationTask:### 고유 값
    def __init__(self):
        self.J_READY = [0, 0, 90, 0, 90, 0]
        self.J_A = [-0.04, 4.78, 60.43, 0.03, 114.86, -0.02]  # A값에서 시작
        self.J_HIGH = 87.0
        self.J_LOW = -176.0
        self.repeat = 3
        self.mj=Move_Js()

    def set_tool_tcp(self):
        set_tool("Tool Weight_3_24")
        set_tcp("TCP208mm")

    def wait_tool(self):
        release()
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while True:
            if (
                check_force_condition(axis=DR_AXIS_Z, max=15, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y, max=15, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X, max=15, ref=DR_TOOL)
            ):
                print('터치 감지 되었습니다.')
                time.sleep(3)
                grip()
                print('잡았습니다.')
                time.sleep(0.5)
                break

    def release_tool(self):
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while True:
            if (
                check_force_condition(axis=DR_AXIS_Z, max=15, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y, max=15, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X, max=15, ref=DR_TOOL)
            ):
                print('터치 감지 되었습니다.')
                time.sleep(3)
                release()
                print('release 성공.')
                time.sleep(0.5)
                break

    def move_to_initial_pose(self): ###고유 함수
        print("[INFO] 비틀기 작업 시작")
        print("[INFO] 초기 자세로 이동")
        self.mj.move_j(self.J_READY, vel=60, acc=ACC)
        print("비틀기를 할 물체를 주세요.")
        self.wait_tool()


    def rotate_twist(self):#### 고유 함수
        for i in range(self.repeat):
            print(f"[INFO] 반복 {i+1}/{self.repeat}")

            print(f"[INFO] J6이 {self.J_HIGH}로 회전")
            j_high = self.J_A.copy()
            j_high[5] = self.J_HIGH
            self.mj.move_j(j_high, vel=VELOCITY, acc=ACC)
            time.sleep(0.5)
            print("[INFO] grip")
            grip()
            time.sleep(1.0)

            print("[INFO] A지점으로 이동")
            self.mj.move_j(self.J_A, vel=VELOCITY, acc=ACC)

            print(f"[INFO] J6을 {self.J_LOW}로 회전")
            j_low = j_high.copy()
            j_low[5] = self.J_LOW
            self.mj.move_j(j_low, vel=VELOCITY, acc=ACC)
            time.sleep(0.5)
            print("[INFO] release")
            release()
            time.sleep(1.0)
            print("[INFO] A지점으로 이동")
            self.mj.move_j(self.J_A, vel=VELOCITY, acc=ACC)
            print(f"[INFO] J6이 {self.J_HIGH}로 회전")
            j_high = self.J_A.copy()
            j_high[5] = self.J_HIGH
            self.mj.move_j(j_high, vel=VELOCITY, acc=ACC)
            time.sleep(0.5)
            print("[INFO] grip")
            grip()
            time.sleep(1.0)
        print("[INFO] 비틀기 반복 완료")
        print("완료 되었습니다. 회수해주세요.")
        self.release_tool()

    def spin_run(self, repeat=3):###고유 실행 함수
        self.repeat=repeat
        self.set_tool_tcp()
        self.move_to_initial_pose()
        self.rotate_twist()
#---------------
#단조 관련 클래스
class Forge():
    def __init__(self):
        self.mj=Move_Js()
    def excep_grab(self):
            while True:
                self.wait_tool()
                time.sleep(0.5)
                a = get_digital_input(1)
                b = get_digital_input(2)
                c = get_digital_input(3)
                # print(f'a:{a}')
                # print(f'b:{b}')
                # print(f'c:{c}')
                if a == 1:
                    print('도구가 정상적으로 장착되지 않았습니다.')
                    # client_socket.sendall(b"Tool grasp failed")
                    print('다시 도구 파지를 시도합니다....')
                    time.sleep(0.2) 
                    self.release50()
                else:
                    print('도구 정상 파지 완료!')
                    # client_socket.sendall(b"Tool grasp sucseesd")
                    break
    def grip15(self):   
        set_digital_output(1,0)
        wait(0.2)
        set_digital_output(2,0)
    def release50(self):
        set_digital_output(1,1)
        wait(0.2)
        set_digital_output(2,0)

    def release_tool(self):
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while(True):
            if(check_force_condition(axis=DR_AXIS_Z,max=20,ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y,max=20,ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X,max=20,ref=DR_TOOL)
                ):
                print('터치 감지 되었습니다.')
                delay_time = 3
                print(f'{delay_time}초 후 놓겠습니다..')
                time.sleep(delay_time)
                
                self.release50()
                print('release 성공.')
                time.sleep(0.5)
                break


    def wait_tool(self):
        self.release50()
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while(True):
            if(check_force_condition(axis=DR_AXIS_Z,max=20,ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y,max=20,ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X,max=20,ref=DR_TOOL)
                ):
                print('터치 감지 되었습니다.')
                delay_time = 3
                print(f'{delay_time}초 후 잡겠습니다.')
                time.sleep(delay_time)
                
                self.grip15()
                print('잡았습니다.')
                time.sleep(0.5)
                break

    def change_tool(self):
        print("self.mj.move_j to waitting point")
        self.mj.move_j(wait_poseJ, vel=VELOCITY, acc=ACC)
        time.sleep(0.5)
        self.release_tool()
        time.sleep(0.5)
        self.excep_grab()


    def time_print(self,num):
        for k in range(1,4):
            print(f'{num}번째 자세 : {k}/3 .......')
            time.sleep(1)


    def holding(self):
        print('모루로 이동하겠습니다.')
        self.mj.move_j(holding1J, vel=VELOCITY, acc=ACC)
        print('3초 주기로 위치 변경하겠습니다.')
        self.time_print(1)
        self.mj.move_j(holding2J, vel=VELOCITY, acc=ACC)
        self.time_print(2)
        self.mj.move_j(holding3J, vel=VELOCITY, acc=ACC)
        self.time_print(3)
        self.mj.move_j(holding4J, vel=VELOCITY, acc=ACC)
        self.time_print(4)
        print('망치질 완료...')
        now = get_current_posx()[0]
        now[2]+=20
        movel(now, vel=VELOCITY, acc=ACC)

    def force_ctrl(self):
        task_compliance_ctrl(stx=[2000, 2000, 100, 20, 20, 20])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        while(True):
            if(check_force_condition(axis=DR_AXIS_Z,max=10,ref=DR_TOOL)):
                print('밀착 성공')
                now = get_current_posx()[0]
                now[2]+=5
                time.sleep(0.1)
                release_force()
                time.sleep(0.1)
                release_compliance_ctrl()
                print('힘 해제')
                time.sleep(0.5)
                movel(now,vel=VELOCITY, acc=ACC)
                print('이동 완료')
                time.sleep(0.5)
                print('사포질 시작')
                task_compliance_ctrl(stx=[100, 100, 100, 20, 20, 20])
                time.sleep(0.1) 
                set_desired_force(fd=[0, 0, -3, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
                
                repeat = 10
                peri = 0.5
                time.sleep(0.1)
                amove_periodic(amp=[0.0, 20.00, 0.00, 0.00, 0.00, 0.00], period=[0.00, 1.00, 0.00, 0.00, 0.00, 0.00], repeat=10 ,ref=0)

                
                time.sleep(repeat*peri+5)

                release_force()
                release_compliance_ctrl()
                print('끝')
                break

    def hammering(self,force):
        time.sleep(0.5)
        task_compliance_ctrl(stx=[20000, 20000, 100, 20, 20, 20])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])

        while(True):
            if(check_force_condition(axis=DR_AXIS_Z,max=10,ref=DR_TOOL)):
                print('밀착 성공')
                now = get_current_posx()[0]
                pos=deepcopy(now)
                pos[2]+=15
                time.sleep(0.1)
                release_force()
                time.sleep(0.1)
                release_compliance_ctrl()
                print('힘 해제')
                time.sleep(0.5)
                movel(pos,vel=VELOCITY, acc=ACC)
                print('이동 완료')
                break

        time.sleep(0.1)
        task_compliance_ctrl(stx=[2000, 2000, 100, 20, 20, 20])
        time.sleep(0.1)
        re_force = 10-force # 1~9
        peri = 0.5 + re_force*0.1
        # peri = 1
        amove_periodic(amp = [0,0,17,0,0,0],period=[0,0,peri,0,0,0], atime=0.2, repeat=10, ref=DR_TOOL)
        time.sleep(10)
        release_compliance_ctrl()
        # release_force()
        print('끝')
        # break

    def play_sapo(self):  # 모루 이동 -> 사포작업 -> z축 살짝 위로 이동
        time.sleep(0.5)
        self.mj.move_j(moru_J, vel=VELOCITY, acc=ACC)
        print('모루 도착')
        time.sleep(0.5)
        self.force_ctrl()

        time.sleep(0.5)
        now1 = get_current_posx()[0]
        # pos=copy(now)
        now1[2]+=50
        time.sleep(0.5)
        movel(now1, vel=VELOCITY, acc=ACC)

    def play_hold(self):  # 모루 이동 -> 잡기 작업 -> 2cm z축 위로
        time.sleep(0.5)
        self.holding()
        time.sleep(0.5)

    def play_hammer(self,num=6):
        time.sleep(0.5)
        self.mj.move_j(moru_J, vel=VELOCITY, acc=ACC)
        self.hammering(int(num))
        time.sleep(0.5)
        now = get_current_posx()[0]
        now[2] += 50
        time.sleep(0.2)
        movel(now, vel=VELOCITY, acc=ACC)
        time.sleep(0.2)

    def ready_tool(self):
        print("self.mj.move_j to home")
        self.mj.move_j(JReady, vel=VELOCITY, acc=ACC)

        print("self.mj.move_j to waitting point")
        self.mj.move_j(wait_poseJ, vel=VELOCITY, acc=ACC)

        self.excep_grab()

    def finsh_tool(self):
        self.mj.move_j(wait_poseJ, vel=VELOCITY, acc=ACC)
        time.sleep(0.5)
        self.release_tool()
        time.sleep(0.5)
        self.mj.move_j(JReady, vel=VELOCITY, acc=ACC)
        print('끝')

    def hammering_start(self,num=8):
        self.ready_tool()
        print('Hammering start!')#기계가 망치질
        self.play_hammer(num)
        self.finsh_tool()

    def Holding_start(self):
        self.ready_tool()
        print('Holding start!') #기계가 잡아줌
        self.play_hold()
        self.finsh_tool()
    
    def vertical_grinding_start(self):
        self.ready_tool()
        print('Vertical Grinding start!')    #갈기
        self.play_sapo()
        self.finsh_tool()

    def all_start(self):
        self.ready_tool()
        print('Hammering start!')#기계가 망치질
        self.play_hammer()
        print('Change tool!!')      
        self.change_tool()
        print('Holding start!') #기계가 잡아줌
        self.play_hold()
        print('Change tool!!')      
        self.change_tool()
        print('Vertical Grinding start!')    #갈기
        self.play_sapo()
        self.finsh_tool()

class TCPServer:
    def __init__(self, host='0.0.0.0', port=1234):
        self.sp = SpitRotationTask()
        self.qc = QCManager()
        self.mcup = MoveCup()
        self.fg = Forge()
        self.gt = GrindingTask()
        self.mj=Move_Js()
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.status = "all"
        self.current_qc_step = 0  # QC 단계 추적
        self.operation_in_progress = False  # 작업 진행 중 여부
        self.command_lock = threading.Lock()
        self.repeat_count = 1
        self.waiting_for_repeat_count = False  # 반복 횟수 입력 대기 상태 추가

    def start(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 서버가 {self.host}:{self.port}에서 시작됨")
        except Exception as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 서버 시작 에러: {e}")
            return

        try:
            while self.running:
                client_socket, addr = self.server_socket.accept()
                client_print_redirector.add_client(client_socket)
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 클라이언트 {addr} 연결됨")
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket, addr))
                client_thread.start()
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 서버 에러: {e}")
        finally:
            self.stop()

    def parse_message(self,message):
        # 콜론(:)이 있는지 확인
        if ':' in message:
            # 콜론으로 분리
            text_part, num_part = message.split(':', 1)
            
            # 숫자 부분을 int로 변환
            number = int(num_part)
            return text_part,number
        
        else:
            # 콜론이 없는 경우: 문자열만 처리
            number = None
            return message,number
        
    def handle_client(self, client_socket, addr):
        try:
            client_socket.send("Enter command (do_all, fd, fg, qc, exit): ".encode('utf-8'))
            while self.running:
                data = client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                print(f"[{timestamp}] 클라이언트 {addr} → 메시지: {data}")
                if(self.status=="all"):
                    response = self.handle_message(data,client_socket)
                elif(self.status=="fd"):
                    response = self.foundry_message(data,client_socket)
                elif(self.status=="fg"):
                    response = self.forge_message(data,client_socket)
                elif(self.status=="qc"):
                    
                    response = self.qc_message(data,client_socket)
        except Exception as e:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(f"[{timestamp}] 클라이언트 {addr} 에러: {e}")
        finally:
            client_socket.close()
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(f"[{timestamp}] 클라이언트 {addr} 연결 종료")

    def foundry_message(self,message,client_socket):
        with self.command_lock:
            if self.operation_in_progress:
                    return "Error: 이미 동작 중입니다."
            msg,num=self.parse_message(message)
            if msg=="casting":
                self.operation_in_progress = True
                self.waiting_for_repeat_count = True  # 반복 횟수 입력 대기 상태
            if self.waiting_for_repeat_count:
                try:
                    count = int(num)
                    if count < 1:
                        return "Error: 반복 횟수는 1 이상의 숫자여야 합니다."
                    self.repeat_count = count
                    self.waiting_for_repeat_count = False  # 대기 상태 해제
                    # 실제 작업은 여기서 수행하거나 따로 사용자가 작업하면 됨
                    self.mcup.run(repeat_m=self.repeat_count)
                    self.operation_in_progress = False
                    return f"반복 횟수 설정 완료: {self.repeat_count}"
                except ValueError:
                    return "Error: 올바른 숫자를 입력해주세요."
            elif msg=="exit":
                self.status="all"
                client_socket.send("Enter command (do_all, fd, fg, qc, exit): ".encode('utf-8'))


    def forge_message(self,message,client_socket):
        with self.command_lock:
            if self.operation_in_progress:
                    return "Error: 이미 동작 중입니다."
            # if message=="hm":
            #     pass
            msg,num=self.parse_message(message)
            print(msg)
            print(num)
            if msg=="all":
                print("all forge")
                self.operation_in_progress = True
                self.fg.all_start()
                self.gt.grinding_run()
                self.sp.spin_run()
                self.operation_sin_progress = False
                return 
            
            elif msg=="hd":
                print("holding")
                self.operation_in_progress = True
                self.fg.Holding_start()
                self.operation_in_progress = False
                return 
            
            elif msg=="hm":
                print("hammering")
                self.operation_in_progress = True
                if(num==None):
                    self.fg.hammering_start()
                else:
                    self.fg.hammering_start(num)
                self.operation_in_progress = False
                return 
            
            elif msg=="vg":
                print("vertical_grinding")
                self.operation_in_progress = True
                self.fg.vertical_grinding_start()
                self.operation_in_progress = False
                return 
            
            elif msg=="hg":
                print("horizontal_grinding")
                self.operation_in_progress = True
                if(num==None):
                    self.gt.grinding_run()
                else:
                    self.gt.grinding_run(num)
                self.operation_in_progress = False
                return 
            
            elif msg=="tw":
                print("tweaking")
                self.operation_in_progress = True
                if(num==None):
                    self.sp.spin_run()
                else:
                    self.sp.spin_run(num)
                self.operation_in_progress = False
                return 
                
            elif msg=="exit":
                self.status="all"
                client_socket.send("Enter command (do_all, fd, fg, qc, exit): ".encode('utf-8'))    ### 단조 과정은 잡기, 해머질, 수평 그라인딩, 수직 그라인딩, 비틀기

    def qc_message(self,message,client_socket):
        with self.command_lock:
            # 명령어 처리
            if self.operation_in_progress:
                    return "Error: 이미 동작 중입니다."
            if message=="auto":
                print("QC_START")
                self.operation_in_progress = True
                self.qc.weiight_classif_auto_movement()
                self.status = "running_qc"
                self.operation_in_progress = False
                return 
            
            elif message=="wc":
                print("masure")
                self.operation_in_progress = True
                self.qc.weight_classification()
                self.operation_in_progress = False
                return 
            
            
            elif message=="exit":
                self.status="all"
                client_socket.send("Enter command (do_all, fd, fg, qc, tw, exit): ".encode('utf-8'))

    def handle_message(self, message,client_socket):
        with self.command_lock:
            # 명령어 처리
            if self.operation_in_progress:
                    return "Error: 이미 동작 중입니다."
            if message == "do_all":
                self.operation_in_progress = True
                self.status = "running_all"
                self.mcup.run()
                self.gt.grinding_run()
                self.sp.spin_run()
                self.fg.all_start()
                self.qc.weiight_classif_auto_movement()
                client_socket.send("Enter command (do_all, fd, fg, qc, exit): ".encode('utf-8'))
                return
                        # casting 명령이 들어오면 반복 횟수 입력 대기 상태로 전환
            elif message == "fd":#foundry
                self.status="fd"
                client_socket.send("Enter command (casting, exit): ".encode('utf-8'))
            elif message == "fg":#forge
                self.status="fg"
                client_socket.send("Enter command (all forge[all], holding[hd], hammering[hm], vertical_grinding[vg], horizontal_grinding[hg], tweaking[tw], exit): ".encode('utf-8'))
            elif message == "qc":
                self.status="qc"
                client_socket.send("Enter command (auto, wc, exit): ".encode('utf-8'))
            elif message=='exit':
                client_socket.close()
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                print(f"[{timestamp}] 클라이언트  연결 종료")

            else:
                return f"Unknown command: {message}"
                        # 반복 횟수 입력 대기 상태에서 숫자 입력 받음
            
            

    def forward_command(self, command):
        """메인 프로그램(robot_control.py)에 명령어 전달"""
        try:
            result = subprocess.run(
                ["python3", "robot_control.py", command],
                capture_output=True,
                text=True,
                check=True
            )
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Forwarded command: {command}, Response: {result.stdout.strip()}")
            return result.stdout.strip() or "Command executed"
        except subprocess.CalledProcessError as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Command error: {e}")
            return f"Error: {e.stderr.strip()}"
        except Exception as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Forward error: {e}")
            return f"Error: {e}"

    def handle_qc_step(self, step):
        steps = {
            "0": "QC_HOME",
            "1": "QC_MOVE_TO_PICK",
            "2": "QC_PICK",
            "3": "QC_MOVE_TO_WEIGH",
            "4": "QC_WEIGH",
            "5-1": "QC_MOVE_TO_DISCARD",
            "5-2": "QC_MOVE_TO_STORE"
        }
        if step in steps:
            if self.operation_in_progress and step != self.current_qc_step:
                return "Error: Operation in progress, please wait or send PAUSE/STOP"
            self.current_qc_step = step
            self.operation_in_progress = True
            response = self.forward_command(steps[step])
            return response
        return f"Invalid QC step: {step}"

    def send_command(self, command):
        """서버 내부에서 명령어 전송 (테스트용)"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(('192.168.0.4', self.port))
                s.send(command.encode('utf-8'))
                response = s.recv(1024).decode('utf-8')
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Sent command: {command}, Response: {response}")
                return response
        except Exception as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Command send error: {e}")
            return f"Error: {e}"

    def stop(self):
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 서버 종료")

class MoveControl:
    def __init__(self, adapt_coords=True):
        """
        MoveControl 클래스를 초기화하며 로봇 설정과 좌표 적응 모드를 설정합니다.
        :param adapt_coords: 좌표 적응 사용 여부 (True: 적응, False: 원본 유지)
        """
        # 좌표 적응 설정
        self.adapt_coords = adapt_coords
        self.mj=Move_Js()
        self.griper = Griper()  # 그리퍼 초기화

    def apply_compliance(self, stx=[COMPLIANCE_AXIS, COMPLIANCE_AXIS, COMPLIANCE_AXIS, COMPLIANCE, COMPLIANCE, COMPLIANCE]):
        """주어진 강성으로 컴플라이언스 제어를 적용합니다."""
        return task_compliance_ctrl(stx=stx)

    def set_force(self, fd, dir, mod=DR_FC_MOD_REL):
        """원하는 힘을 설정합니다."""
        return set_desired_force(fd=fd, dir=dir, mod=mod)

    def check_force(self, axis, max_force, ref=DR_BASE):
        """지정된 축에서 힘 조건을 확인합니다."""
        return check_force_condition(axis=axis, max=max_force, ref=ref)

    def release_compliance(self):
        """컴플라이언스 제어를 해제합니다."""
        return release_compliance_ctrl()

    def release_force(self):
        """힘 제어를 해제합니다."""
        return release_force()

    def is_position_reached(self, target_pos):
        """현재 위치가 목표 위치의 허용 오차 내에 있는지 확인합니다."""
        current_pos = self.get_current_pos()
        for i in range(3):  # x, y, z 좌표 확인
            if abs(current_pos[i] - target_pos[i]) > self.pos_tolerance:
                return False
        return True

    def apply_force_control(self, axis, direction, pos):
        """
        픽 앤 플레이스를 위한 힘 제어를 적용합니다.
        :param axis: 힘 적용 축 (DR_AXIS_X, Y, Z)
        :param direction: 힘 방향 (+1 또는 -1)
        :param pos: 목표 위치
        :return: (성공 여부, 위치) - 성공 플래그와 적응/원본 위치
        """
        print(f"go to auto grip pos: {pos}")
        time.sleep(1.5)
        movel(pos, vel=DR_VEL_L, acc=DR_ACC_L,ref=DR_BASE)
        time.sleep(0.5)
        print("compliance on")
        self.apply_compliance()
        time.sleep(0.5)
        fd = [0]*6; fd[axis] = TARGET_FORCE * direction
        dir = [0]*6; dir[axis] = 1
        time.sleep(0.5)
        print("force on")
        self.set_force(fd=fd, dir=dir)
        time.sleep(0.5)
        while True:
            if(self.check_force(axis, CHECK_FORCE)):
                print("find pos reease force and compliance")
                release_force()
                time.sleep(0.5)
                release_compliance_ctrl()
                time.sleep(0.5)
                wait(0.5)
                pos_x = get_current_posx()[0]
                print(f"now pos = {pos_x}")
                pos_change = deepcopy(pos_x)
                pos_change[axis] += 100.0*(direction*-1)
                print("move back")
                time.sleep(0.5)
                movel(pos_x, vel=DR_VEL_L, acc=DR_ACC_L,ref=DR_BASE)
                time.sleep(0.5)
                break
        return pos_x

    def auto_grip_cup(self,pos_j, start_pos, axis = DR_AXIS_Y, direction = -1, grip_type="cup"):
        """
        힘 제어를 포함한 픽 앤 플레이스 작업을 수행합니다.
        :param pos_j: 초기관절위치 (posj)
        :param start_pos: 시작 위치 (posx)
        :param axis: 힘 적용 축 (DR_AXIS_X, Y, Z)
        :param direction: 힘 방향 (+1 또는 -1)
        :param grip_type: 그리퍼 타입 ('standard' 또는 'cup')
        :return: 성공 시 True, 실패 시 False
        """
        # 초기 관절 위치로 이동
        print("pos cup station")
        self.mj.move_j(pos_j,acc=DR_ACC_J,vel=DR_VEL_J)

        # 힘 제어 적용 target_pos는 원래 자리에 되돌려 놓을걸 가정하고 미리 받아서 나중에 활용 할까 생각중인데 안써도 그대로 둘듯
        print("start auto grip pos process")
        target_pos = self.apply_force_control(axis, direction, start_pos)

        # 그리퍼 해제
        print("release grip")
        
        time.sleep(1.5)
        self.griper.release_cup()
        time.sleep(0.5)

        print("move grip pos")
        move_pos = deepcopy(target_pos)
        if(grip_type=="cup"):
            move_pos[axis] += direction*70.0
        else:
            move_pos[axis] += direction*10.0
        time.sleep(1.5)
        movel(move_pos,acc=DR_ACC_L,vel=DR_VEL_L,ref=DR_BASE)

        print("grip")
        # 그리퍼 동작
        time.sleep(1.5)
        self.griper.grip_cup()
        time.sleep(3)
        move_pos[2]+=50
        movel(move_pos,acc=DR_ACC_L,vel=DR_VEL_L,ref=DR_BASE)
        time.sleep(3)

class POS:
    def __init__(self, pos_dict=None, file_path="src/DoosanBootcamInt1/dsr_rokey/rokey/rokey/basic/data/pos/spots.json"):
        self.spots = {}  # {"name": {POS_X: [x,y,z,a,b,c], POS_J: [j1,j2,j3,j4,j5,j6]}}
        self.file_path = file_path

        # pos_dict가 제공되면 처리 및 저장
        if pos_dict is not None:
            self.add_spots_from_dict(pos_dict)
            self.save_spots()
        # pos_dict가 없으면 파일에서 로드 시도
        else:
            print("pos를 읽어오는 중입니다.")
            self.load_spots()

    def set_j(self, pos_x, solspace=4):
        """pos_x를 입력받아 pos_j(관절 좌표)를 계산"""
        try:
            j = ikin(pos_x, solspace)
            return j.tolist() if isinstance(j, np.ndarray) else j
        except Exception as e:
            print(f"ikin error: {e}")
            return None

    def add_spot(self, name, pos_x, solspace=2):
        """이름과 pos_x를 받아 pos_j를 계산하고 저장"""
        if not isinstance(pos_x, list) or len(pos_x) != 6:
            print(f"Error: pos_x {pos_x} must be a list of 6 elements [x,y,z,a,b,c]")
            return False
        
        pos_j = self.set_j(pos_x, solspace)
        if pos_j is None:
            print(f"Failed to calculate pos_j for {name}")
            return False

        self.spots[name] = {POS_X: pos_x, POS_J: pos_j}
        print(f"Added spot {name}: pos_x={pos_x}, pos_j={pos_j}")
        return True

    def add_spots_from_list(self, pos_list, name_prefix="pos"):
        """리스트 형태 [[x,y,z,a,b,c], ...]로 여러 스팟 추가"""
        for i, pos_x in enumerate(pos_list):
            name = f"{name_prefix}_{i+1}_spot"
            self.add_spot(name, pos_x)

    def add_spots_from_dict(self, pos_dict):
        """딕셔너리 형태 {name: [x,y,z,a,b,c], ...}로 여러 스팟 추가"""
        for name, pos_x in pos_dict.items():
            self.add_spot(name, pos_x)

    def get_pos_x(self, name):
        """이름으로 pos_x 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_X]

    def get_pos_j(self, name):
        """이름으로 pos_j 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_J]

    def get_spot(self, name):
        """이름으로 pos_x와 pos_j 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_X], spot[POS_J]

    def get_all_spots(self):
        """모든 스팟 정보 리턴"""
        return self.spots

    def save_spots(self):
        """spots 데이터를 JSON 파일로 저장"""
        try:
            with open(self.file_path, "w") as f:
                json.dump(self.spots, f, indent=4)
            print(f"Spots saved to {self.file_path}")
        except Exception as e:
            print(f"Error saving spots to {self.file_path}: {e}")

    def load_spots(self):
        """JSON 파일에서 spots 데이터 로드"""
        try:
            with open(self.file_path, "r") as f:
                self.spots = json.load(f)
            print(f"Spots loaded from {self.file_path}")
            print(f"spot={self.spots}")
        except FileNotFoundError:
            print(f"Error: File {self.file_path} not found")
            self.spots = {}
        except Exception as e:
            print(f"Error loading spots from {self.file_path}: {e}")
            self.spots = {}


class WeightMeasurement:
    def __init__(self, sequence_number):
        """무게 측정 클래스 초기화 with sequence number"""
        self.sequence_number = sequence_number
        self.csv_file = self._get_unique_csv_filename()
        self.mj=Move_Js()

    def _get_unique_csv_filename(self):
        """고유한 CSV 파일 이름 생성 (weight_data_YY_MM_DD_X.csv)"""
        base_path = "src/DoosanBootcamInt1/dsr_rokey/rokey/rokey/basic/data/weight"
        os.makedirs(base_path, exist_ok=True)  # 경로가 없으면 생성
        now = datetime.now()
        date_str = now.strftime("%y_%m_%d")
        base_name = f"weight_data_{date_str}"
        counter = 0
        csv_file = os.path.join(base_path, f"{base_name}_{counter}.csv")
        
        while os.path.exists(csv_file):
            counter += 1
            csv_file = os.path.join(base_path, f"{base_name}_{counter}.csv")
        
        return csv_file

    def measure_workpiece(self, samples=100):
        """get_workpiece_weight로 무게(g)를 여러 번 측정해 평균 반환"""
        time.sleep(2)
        weights = []
        for _ in range(samples):
            weight = get_workpiece_weight()
            if weight != -1:  # 성공 시
                w=deepcopy(weight)
                print(w*1000+30)
                weights.append(weight * 1000+30)
                
            wait(0.01)  # 측정 간 0.01초 대기
        return sum(weights) / len(weights) if weights else -1

    def save_to_csv(self, weight, method, target=212):
        """무게 데이터를 CSV로 저장 with sequence number"""
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H:%M:%S")
        error = abs(weight - target) if weight != -1 else -1
        with open(self.csv_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            # Write header if file is new
            if os.path.getsize(self.csv_file) == 0:
                writer.writerow(["Date", "Time", "Sequence", "Weight", "Method", "Quality", "Error"])
            writer.writerow([date_str, time_str, self.sequence_number, weight, method, error])

    # 아래 방식은 정확도를 올릴 수 있지만 외력 영향에 수정이 어려워 외력 영향에도 약간의 보정이 있는 기본 제공 을 사용하기로 결정
    def measure_tool_force_z(self, samples=5, ref=0):
        """get_tool_force로 Z축 힘(N)만 사용해 무게(g) 평균 반환"""
        weights = []
        for _ in range(samples):
            forces = get_tool_force(ref=ref)
            if forces != -1:  # 성공 시
                fz = forces[2]  # Z축 힘 (N)
                weight = fz / 9.81 * 1000  # N → g 변환
                weights.append(weight)
            wait(0.1)  # 측정 간 0.1초 대기
        return sum(weights) / len(weights) if weights else -1

    def measure_tool_force_6axis(self, samples=5, ref=0):
        """get_tool_force로 6축 데이터를 사용해 Z축 방향 힘으로 무게(g) 평균 반환"""
        weights = []
        for _ in range(samples):
            forces = get_tool_force(ref=ref)
            if forces != -1:  # 성공 시
                fx, fy, fz, _, _, _ = forces  # Fx, Fy, Fz (N), 토크는 무시
                # Z축 방향으로 합성된 힘 계산 (단순 벡터 합으로 근사)
                f_total = math.sqrt(fx**2 + fy**2 + fz**2)
                # Z축 비율로 무게 추정
                weight = (fz / f_total * f_total) / 9.81 * 1000 if f_total != 0 else fz / 9.81 * 1000
                weights.append(weight)
            wait(0.1)  # 측정 간 0.1초 대기
        return sum(weights) / len(weights) if weights else -1

class Griper:
    def __init__(self):
        pass
        


    def grip(self):
        self.release()
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

    def release(self):
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        
        
    def grip_cup(self):
        self.release_cup()
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def release_cup(self):
        set_digital_output(1, ON)
        set_digital_output(2, ON)

    # 수정해야함 제대로 이해 안되서 로직이 안됨. 빈상태로 잡았을 때를 읽어와서 에러 상태로 했으면 좋겠긴함.
    def read_input(self, index):
        return get_digital_input(index)

spots = {
    "starting_point_for_placing_defective_items": [620.880, -396.210, 75.380, 92.62, -90.27, -89.62],
    "placing_defective_items": [291.230, -534.300, 220.520, 92.47, -89.87, -91.89],
    "weight_measurement_position": [620.050, -316.670, 223.090, 91.71, -89.53, 86.24],
    "place_position_if_weight_ok": [280.960, -121.090, 133.720, 92.37, -89.73, -88.39],
    "cup_grab_position":  [623.060, -363.060, 506.030, 88.75, -91.66, -91.9],
    "first_cup_position": [625.560, -461.170, 484.090, 90.76, -91.97, -91.37],
    "second_cup_position": [614.320, -582.500, 480.630, 93.29, -71.74, -89.96]
}

class QCManager:
    def __init__(self):
        self.ps = POS()
        self.gr = Griper()
        self.wm = WeightMeasurement(sequence_number=1)
        self.mc = MoveControl()
        self.mj=Move_Js()
        self.status = "idle"
        self.current_qc_step = 0
        self.paused = False
        self.operation_in_progress = False

    def movement_item_measure_loc(self, posj, posx):
        print("start cup auto grip...")
        self.mc.auto_grip_cup(posj, posx)
        print("move up pos grip to pos start")
        time.sleep(0.5)
        pos = self.ps.get_pos_x("cup_grab_position")
        pos[2]+=50
        time.sleep(0.5)
        movel(pos, vel=DR_VEL_L, acc=DR_ACC_L)
        time.sleep(0.5)
        print("move measure position")
        self.mj.move_j(self.ps.get_pos_j("weight_measurement_position"), vel=DR_VEL_J, acc=DR_ACC_J)
        time.sleep(0.5)

    def movement_item_loc(self, thrown):
        if thrown==False:
            print("move start thrown pos")
            self.mj.move_j(self.ps.get_pos_j("starting_point_for_placing_defective_items"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(0.5)
            print("move thrown pos")
            self.mj.move_j(self.ps.get_pos_j("placing_defective_items"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(1.5)
            self.gr.release_cup()
            time.sleep(0.5)
            print("move start thrown pos")
            self.mj.move_j(self.ps.get_pos_j("starting_point_for_placing_defective_items"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(0.5)
        else:
            print("move store pos")
            self.mj.move_j(self.ps.get_pos_j("place_position_if_weight_ok"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(0.5)
            pos=deepcopy(self.ps.get_pos_x("place_position_if_weight_ok"))
            pos[2]-=20
            time.sleep(0.5)
            movel(pos,vel=DR_VEL_L,acc=DR_ACC_L)
            time.sleep(1.5)
            self.gr.release_cup()
            time.sleep(0.5)
            self.mj.move_j(self.ps.get_pos_j("place_position_if_weight_ok"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(0.5)
            

    def weight_classification(self):
        time.sleep(5)
        # reset_workpiece_weight()
        weight1 = self.wm.measure_workpiece()
        time.sleep(1.5)
        if weight1 != -1:
            # 무게 보정
            print(f"Workpiece Weight: {weight1:.2f}g")
            time.sleep(1.5)
            self.wm.save_to_csv(weight1, "workpiece")
            time.sleep(1.5)
        else:
            print("Workpiece Weight: 측정 실패")

            # # get_tool_force로 Z축만 사용
            # weight2 = self.wm.measure_tool_force_z()
            # if weight2 != -1:
            #     print(f"Tool Force (Z-axis): {weight2:.2f}g")
            #     self.wm.save_to_csv(weight2, "tool_force_z")
            # else:
            #     print("Tool Force (Z-axis): 측정 실패")

            # # get_tool_force로 6축 데이터 사용
            # weight3 = self.wm.measure_tool_force_6axis()
            # if weight3 != -1:
            #     print(f"Tool Force (6-axis): {weight3:.2f}g")
            #     self.wm.save_to_csv(weight3, "tool_force_6axis")
            # else:
            #     print("Tool Force (6-axis): 측정 실패")
            # num+=1
            # wait(1.0)  # 다음 측정까지 1초 대기
            # 제일 무거운거 211, 그다음 178
        return 201 <= weight1 <= 221

    def weiight_classif_auto_movement(self):
        print("set_init")
        self.gr.release_cup()
        time.sleep(1.5)
        self.mj.move_j([0,0,90,0,90,0],vel=DR_VEL_J,acc=DR_ACC_J)
        time.sleep(1.5)
        self.gr.grip()
        time.sleep(1.5)
        flag = [False, False]
        self.init_pos_measure()
        time.sleep(1.5)
        self.movement_item_measure_loc(self.ps.get_pos_j("cup_grab_position"), self.ps.get_pos_x("first_cup_position"))
        flag[0] = self.weight_classification()
        self.movement_item_loc(flag[0])
        time.sleep(1.5)
        self.gr.grip()
        time.sleep(3)
        self.movement_item_measure_loc(self.ps.get_pos_j("cup_grab_position"), self.ps.get_pos_x("second_cup_position"))
        time.sleep(1.5)
        flag[1] = self.weight_classification()
        time.sleep(1.5)
        self.movement_item_loc(flag[1])
        self.operation_in_progress = False
        self.status = "idle"
        return "QC completed"
    
    def init_pos_measure(self):
        self.gr.release_cup()
        time.sleep(3)
        self.mj.move_j(self.ps.get_pos_j("weight_measurement_position"),vel=DR_VEL_J,acc=DR_ACC_J)
        time.sleep(3)
        reset_workpiece_weight()
        time.sleep(4)
        self.gr.grip()
        time.sleep(3)

    def monitor_robot(self):
        """로봇팔 상태 모니터링 (예: 외력 감지)"""
        while True:
            try:
                force = get_tool_force(DR_BASE)  # 외력 측정
                if any(abs(f) > CHECK_FORCE for f in force[:3]) and not self.paused:
                    self.handle_command("PAUSE")
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 외력 감지: 로봇 일시 중지")
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 모니터링: 상태={self.status}, QC 단계={self.current_qc_step}")
                time.sleep(1)
            except Exception as e:
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 모니터링 에러: {e}")
                time.sleep(1)

def main():
    server = TCPServer()
    server.start()
    

if __name__ == "__main__":
    main()