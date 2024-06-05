#!/usr/bin/env python3
#-*-coding:utf-8-*-

# 2024버전 state

# Python packages

import rospy
import os
import numpy as np
import sympy as sp
from math import cos, sin
import dynamixel_sdk
from dynamixel_sdk.controllerXC import *
from dynamixel_sdk.controllerAX import *
from trajectory_planning import *

MISSION = "CUBE" # "CUBE", "CIRCLE"



"""self.angle_array = Float64MultiArray()
Configuration for keyboard interrupt
"""
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


##########################################################################################################################
# Initialize parameters
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
BAUDRATE                    = 115200
AX1_ID                      = 0                 # ID of AX motor
AX2_ID                      = 1                 # ID of AX motor
AX3_ID                      = 2                 # ID of AX motor
AX4_ID                      = 3                 # ID of AX motor
XC1_ID                      = 4                 # ID of XC motor
XC_MOVING_STATUS_THRESHOLD  = 10
AX_MOVING_STATUS_THRESHOLD  = 10
ADDR_COMPLIANCE_MARGIN_CW = 26
ADDR_COMPLIANCE_MARGIN_CCW = 27
ADDR_COMPLIANCE_SLOPE_CW = 28  # Compliance Slope CW 주소
ADDR_COMPLIANCE_SLOPE_CCW = 29  # Compliance Slope CCW 주소
COMPLIANCE_SLOPE_VALUE = 128

PEN_FPS = 40
PEN_SPEED = 100

CUBE_FPS = 20
CUBE_SPEED = 100

# Set port handler
portHandler = PortHandler(DEVICENAME)
packetHandler = dynamixel_sdk.PacketHandler(1.0)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()



def calc_torque():
    sp.init_printing(use_unicode=True)

    # Set Lagragian equation
    link_length = 0.13 # m
    pen_gripper_length = 0.005
    cube_gripper_length = 0.06
    link_weight = 18.8e-3
    ax18a_weight = 55.9e-3 # kg
    xc330_weight = 23.0e-3 # kg
    pen_gripper_weight = 40.0e-3 # kg
    cube_gripper_weight = 41e-3 # kg
    g = 9.81
    o1, o2, o3 = sp.symbols('o1, o2, o3')
    if MISSION == "CIRCLE": 
        Tl1 = link_weight * g * (link_length/2 * sp.sin(o1))
        Tm1 = ax18a_weight * g * (link_length * sp.sin(o1))
        Tl2 = link_weight * g * (link_length * sp.sin(o1) + link_length/2 * sp.sin(o1+o2))
        Tm2 = ax18a_weight * g * (link_length * sp.sin(o1) + link_length * sp.sin(o1+o2))
        Tgrip = pen_gripper_weight * g * (link_length * sp.sin(o1) + link_length * sp.sin(o1+o2) + pen_gripper_length*sp.sin(o1+o2+o3))
        T_motor1 = Tl1 + Tm1 + Tl2 + Tm2 + Tgrip
        Tl2 = link_weight * g * (link_length/2 * sp.sin(o1+o2))
        Tm2 = ax18a_weight * g * (link_length * sp.sin(o1+o2))
        Tgrip = pen_gripper_weight * g * (link_length * sp.sin(o1+o2) + pen_gripper_length*sp.sin(o1+o2+o3))
        T_motor2 = Tl2 + Tm2 + Tgrip
        Tgrip = pen_gripper_weight * g * (pen_gripper_length*sp.sin(o1+o2+o3))
        T_motor3 = Tgrip
        return T_motor1, T_motor2, T_motor3
    elif MISSION == "CUBE":
        Tl1 = link_weight * g * (link_length/2*sp.sin(o1))
        Tm1 = ax18a_weight * g * (link_length*sp.sin(o1))
        Tl2 = link_weight * g * (link_length * sp.sin(o1) + link_length/2 * sp.sin(o1+o2))
        Tm2 = ax18a_weight * g * (link_length * sp.sin(o1) + link_length * sp.sin(o1+o2))
        Tgrip = cube_gripper_weight * g * (link_length * sp.sin(o1) + link_length * sp.sin(o1+o2) + (cube_gripper_length+xc330_weight)*sp.sin(o1+o2+o3))
        T_motor1 = Tl1 + Tm1 + Tl2 + Tm2 + Tgrip
        Tl2 = link_weight * g * (link_length/2 * sp.sin(o1+o2))
        Tm2 = ax18a_weight * g * (link_length * sp.sin(o1+o2))
        Tgrip = pen_gripper_weight * g * (link_length * sp.sin(o1+o2) + pen_gripper_length*sp.sin(o1+o2+o3))
        T_motor2 = Tl2 + Tm2 + Tgrip
        Tgrip = pen_gripper_weight * g * (pen_gripper_length*sp.sin(o1+o2+o3))
        T_motor3 = Tgrip
        return T_motor1, T_motor2, T_motor3
##########################################################################################################################

class Angle_Controller():
    def __init__(self, portHandler, trajectory, omega, T_motor1, T_motor2, T_motor3):

        self.omega = omega.astype(int)
        self.T_motor1 = T_motor1
        self.T_motor2 = T_motor2
        self.T_motor3 = T_motor3

        if MISSION == "CIRCLE":
            self.AX1_controller = ControllerAX(portHandler, BAUDRATE, AX1_ID)
            self.AX1_controller.torque_enable()
            self.AX1_controller.set_moving_speed(speed_value=PEN_SPEED)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX1_ID, ADDR_COMPLIANCE_SLOPE_CW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX1_ID, ADDR_COMPLIANCE_SLOPE_CCW, COMPLIANCE_SLOPE_VALUE)
            self.ax1_pos = 512                       # Set initial position of AX motor    
            self.AX2_controller = ControllerAX(portHandler, BAUDRATE, AX2_ID)
            self.AX2_controller.torque_enable()
            self.AX2_controller.set_moving_speed(speed_value=PEN_SPEED)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX2_ID, ADDR_COMPLIANCE_SLOPE_CW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX2_ID, ADDR_COMPLIANCE_SLOPE_CCW, COMPLIANCE_SLOPE_VALUE)
            self.ax2_pos = 512                     # Set initial position of AX motor    
            self.AX3_controller = ControllerAX(portHandler, BAUDRATE, AX3_ID)
            self.AX3_controller.torque_enable()
            self.AX3_controller.set_moving_speed(speed_value=PEN_SPEED)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX3_ID, ADDR_COMPLIANCE_SLOPE_CW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX3_ID, ADDR_COMPLIANCE_SLOPE_CCW, COMPLIANCE_SLOPE_VALUE)
            self.ax3_pos = 512                       # Set initial position of AX motor    
            self.AX4_controller = ControllerAX(portHandler, BAUDRATE, AX4_ID)
            self.AX4_controller.torque_enable()
            self.AX4_controller.set_moving_speed(speed_value=PEN_SPEED)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX4_ID, ADDR_COMPLIANCE_SLOPE_CW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX4_ID, ADDR_COMPLIANCE_SLOPE_CCW, COMPLIANCE_SLOPE_VALUE)
            self.ax4_pos = 512                       # Set initial position of AX motor
            self.AX1_controller.move(512)       # Move AX motor
            self.AX2_controller.move(512)       # Move AX motor
            self.AX3_controller.move(512)       # Move AX motor
            self.AX4_controller.move(512)       # Move AX motor
            time.sleep(2)

        elif MISSION == "CUBE":
            self.AX1_controller = ControllerAX(portHandler, BAUDRATE, AX1_ID)
            self.AX1_controller.torque_enable()
            self.AX1_controller.set_moving_speed(speed_value=CUBE_SPEED)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX1_ID, ADDR_COMPLIANCE_SLOPE_CW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX1_ID, ADDR_COMPLIANCE_SLOPE_CCW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX1_ID, ADDR_COMPLIANCE_MARGIN_CW, 1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX1_ID, ADDR_COMPLIANCE_MARGIN_CCW, 1)
            self.ax1_pos = 512                       # Set initial position of AX motor    
            self.AX2_controller = ControllerAX(portHandler, BAUDRATE, AX2_ID)
            self.AX2_controller.torque_enable()
            self.AX2_controller.set_moving_speed(speed_value=CUBE_SPEED)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX2_ID, ADDR_COMPLIANCE_SLOPE_CW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX2_ID, ADDR_COMPLIANCE_SLOPE_CCW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX2_ID, ADDR_COMPLIANCE_MARGIN_CW, 1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX2_ID, ADDR_COMPLIANCE_MARGIN_CCW, 1)
            self.ax2_pos = 512                     # Set initial position of AX motor    
            self.AX3_controller = ControllerAX(portHandler, BAUDRATE, AX3_ID)
            self.AX3_controller.torque_enable()
            self.AX3_controller.set_moving_speed(speed_value=CUBE_SPEED)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX3_ID, ADDR_COMPLIANCE_SLOPE_CW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX3_ID, ADDR_COMPLIANCE_SLOPE_CCW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX3_ID, ADDR_COMPLIANCE_MARGIN_CW, 1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX3_ID, ADDR_COMPLIANCE_MARGIN_CCW, 1)
            self.ax3_pos = 512                       # Set initial position of AX motor    
            self.AX4_controller = ControllerAX(portHandler, BAUDRATE, AX4_ID)
            self.AX4_controller.torque_enable()
            self.AX4_controller.set_moving_speed(speed_value=CUBE_SPEED)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX4_ID, ADDR_COMPLIANCE_SLOPE_CW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX4_ID, ADDR_COMPLIANCE_SLOPE_CCW, COMPLIANCE_SLOPE_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX4_ID, ADDR_COMPLIANCE_MARGIN_CW, 1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AX4_ID, ADDR_COMPLIANCE_MARGIN_CCW, 1)
            self.ax4_pos = 512                       # Set initial position of AX motor
            self.XC1_controller = ControllerXC(portHandler, BAUDRATE, XC1_ID)
            self.XC1_controller.torque_enable()
            self.XC1_controller.set_position_p_gain(speed_value=100)
            self.xc1_pos = 3186                      # Set initial position of XC motor
            self.AX1_controller.move(512)       # Move AX motor
            self.AX2_controller.move(512)       # Move AX motor
            self.AX3_controller.move(512)       # Move AX motor
            self.AX4_controller.move(512)       # Move AX motor
            self.XC1_controller.move(3186)
            time.sleep(2)

        self.t1_offset = 0
        self.t2_offset = 0
        self.t3_offset = 0
        self.call_offset()
        self.trajectory = trajectory

        self.link_length = 0.13 # m
        self.link_weight = 18.8e-3
        self.ax18a_weight = 55.9e-3 # kg
        self.xc330_weight = 23.0e-3 # kg
        self.pen_gripper_weight = 40.0e-3 # kg
        self.cube_gripper_weight = 41e-3 # kg

    def call_offset(self):
        if MISSION == "CIRCLE": 
            self.t1_offset = 7/(-0.3658)
            self.t2_offset = 5/(-0.1363)
            self.t3_offset = 2/(-0.002)
        elif MISSION == "CUBE":
            self.t1_offset = 14/(-0.3998)
            self.t2_offset = 7/(-0.13625)
            self.t3_offset = 4/(-0.001962)

    def omega_change(self, omega):
        
        self.AX1_controller.set_moving_speed(speed_value=abs(omega[0]+1))
        self.AX2_controller.set_moving_speed(speed_value=abs(omega[1]+1))
        self.AX3_controller.set_moving_speed(speed_value=abs(omega[2]+1))
        self.AX4_controller.set_moving_speed(speed_value=abs(omega[3]+1))

    def pen_tracking(self):
        prev_time = time.time()
        o1, o2, o3 = sp.symbols('o1, o2, o3')
        for i, angles in enumerate(self.trajectory):
            new_time = time.time()
            # os.system('clear')
            # print(angles * 180 /np.pi)
            # angle1 = 0
            # angle2 = 0
            # angle3 = 0
            # angle4 = 0
            # angles = np.array([angle1, angle2, angle3, angle4])
            T1 = self.T_motor1.subs([(o1, angles[1]),(o2, angles[2]),(o3, angles[3])])
            T2 = self.T_motor2.subs([(o1, angles[1]),(o2, angles[2]),(o3, angles[3])])
            T3 = self.T_motor3.subs([(o1, angles[1]),(o2, angles[2]),(o3, angles[3])])

            ax1_offset = int(self.t1_offset * T1 * 1.4 + 6)
            ax2_offset = int(self.t2_offset * T2 * 1.4 + 4)
            ax3_offset = int(self.t3_offset * T3 * 1.4)
            # print(ax1_offset, ax2_offset, ax3_offset)
            # T1 = self.T_motor1.subs([(o1, angle2),(o2, angle3),(o3, angle4)])
            # T2 = self.T_motor2.subs([(o1, angle2),(o2, angle3),(o3, angle4)])
            # T3 = self.T_motor3.subs([(o1, angle2),(o2, angle3),(o3, angle4)])
            
            # print("T_motor1: ", T1)
            # print("T_motor2: ", T2)
            # print("T_motor3: ", T3)
            
            angle_array = angles * 616 / np.pi + 512
            angle_array[1] = angle_array[1] + ax1_offset
            angle_array[2] = angle_array[2] + ax2_offset
            angle_array[3] = angle_array[3] + ax3_offset
            angle_array = angle_array.astype(int)

            #print("goal: ",angle_array)
            # m1 = self.AX1_controller.read_current_position()
            # m2 = self.AX2_controller.read_current_position()
            # m3 = self.AX3_controller.read_current_position()
            # m4 = self.AX4_controller.read_current_position()
            
            
            # self.AX1_controller.move(angle_array[0])       # Move AX motor
            # self.AX2_controller.move(angle_array[1])       # Move AX motor
            # self.AX3_controller.move(angle_array[2])       # Move AX motor
            # self.AX4_controller.move(angle_array[3])       # Move AX motor
            

            self.AX1_controller.move(angle_array[0])       # Move AX motor
            self.AX2_controller.move(angle_array[1])       # Move AX motor
            self.AX3_controller.move(angle_array[2])       # Move AX motor
            self.AX4_controller.move(angle_array[3])       # Move AX motor
            m_offset = [0,0,0,0]
            # while(True):
                
            #     m1 = self.AX1_controller.read_current_position()
            #     m2 = self.AX2_controller.read_current_position()
            #     m3 = self.AX3_controller.read_current_position()
            #     m4 = self.AX4_controller.read_current_position()

            #     m = [m1, m2, m3, m4]
            #     print(m)
            #     for i in range(0, 4):
            #         if angle_array[i] > m[i]:
            #             m_offset[i] = 1
            #         elif angle_array[i] < m[i]:
            #             m_offset[i] = -1
            #         else:
            #             m_offset[i] = 0

            #     print(m_offset[0], m_offset[1], m_offset[2], m_offset[3])
            #     if abs(m1 - angle_array[0]) <= 1 and abs(m1 - angle_array[1]) <= 1 and abs(m1 - angle_array[2]) <= 1 and abs(m1 - angle_array[3]) <= 1:
            #         print("goal: ", angle_array)
            #         print("real: ",[m1, m2, m3, m4])
            #         break

            #     for i in range(0, 4):
            #         angle_array[i] += m_offset[i]
            
            next_time = prev_time + (1/CUBE_SPEED)
            while(next_time-time.time() > 0):
                m1 = self.AX1_controller.read_current_position()
                m2 = self.AX2_controller.read_current_position()
                m3 = self.AX3_controller.read_current_position()
                m4 = self.AX4_controller.read_current_position()
                if (abs(m1 - angle_array[0]) < AX_MOVING_STATUS_THRESHOLD
                and abs(m2 - angle_array[1]) < AX_MOVING_STATUS_THRESHOLD
                and abs(m3 - angle_array[2]) < AX_MOVING_STATUS_THRESHOLD
                and abs(m4 - angle_array[3]) < AX_MOVING_STATUS_THRESHOLD
                ):
                    print("check")
                    break    

            # XC1_controller.move(angle_array[4])       # 무조건 1850 ~ 2700
            # next_time = prev_time + (i+1) * (1/PEN_FPS)
            # print(time.time()- new_time)
            # time.sleep(max(0,next_time-time.time()))
    
    def cube_tracking(self):
        prev_time = time.time()
        o1, o2, o3 = sp.symbols('o1, o2, o3')
        
        for i, angles in enumerate(self.trajectory):
            prev_time = time.time()

            if angles[0] == 100: # up
                time.sleep(1)
                self.XC1_controller.move(3945)       # 무조건 3186 ~ 3945
                time.sleep(1)
                prev_time +=2
                continue

            elif angles[0] == -100: # down
                time.sleep(1)
                self.XC1_controller.move(3186)       # 무조건 3186 ~ 3945
                time.sleep(1)
                prev_time +=2
                continue

            T1 = self.T_motor1.subs([(o1, angles[1]),(o2, angles[2]),(o3, angles[3])])
            T2 = self.T_motor2.subs([(o1, angles[1]),(o2, angles[2]),(o3, angles[3])])
            T3 = self.T_motor3.subs([(o1, angles[1]),(o2, angles[2]),(o3, angles[3])])

            ax1_offset = int(self.t1_offset * T1 * 1.0 +1)
            ax2_offset = int(self.t2_offset * T2 * 1.0 +1)
            ax3_offset = int(self.t3_offset * T3 * 1)
            # print(ax1_offset, ax2_offset, ax3_offset)
            # T1 = self.T_motor1.subs([(o1, angle2),(o2, angle3),(o3, angle4)])
            # T2 = self.T_motor2.subs([(o1, angle2),(o2, angle3),(o3, angle4)])
            # T3 = self.T_motor3.subs([(o1, angle2),(o2, angle3),(o3, angle4)])
            
            # print("T_motor1: ", T1)
            # print("T_motor2: ", T2)
            # print("T_motor3: ", T3)
            
            angle_array = angles * 616 / np.pi + 512
            angle_array[1] = angle_array[1] + ax1_offset
            angle_array[2] = angle_array[2] + ax2_offset
            angle_array[3] = angle_array[3] + ax3_offset
            angle_array = angle_array.astype(int)

            #print("goal: ",angle_array)
            # m1 = self.AX1_controller.read_current_position()
            # m2 = self.AX2_controller.read_current_position()
            # m3 = self.AX3_controller.read_current_position()
            # m4 = self.AX4_controller.read_current_position()
            #print("real: ",[m1, m2, m3, m4])

            self.AX1_controller.move(angle_array[0]+20)       # Move AX motor
            self.AX2_controller.move(angle_array[1])       # Move AX motor
            self.AX3_controller.move(angle_array[2])       # Move AX motor
            self.AX4_controller.move(angle_array[3])       # Move AX motor
            
            next_time = prev_time + (1/CUBE_SPEED)
            while(next_time-time.time() > 0):
                m1 = self.AX1_controller.read_current_position()
                m2 = self.AX2_controller.read_current_position()
                m3 = self.AX3_controller.read_current_position()
                m4 = self.AX4_controller.read_current_position()
                if (abs(m1 - angle_array[0]) < AX_MOVING_STATUS_THRESHOLD
                and abs(m2 - angle_array[1]) < AX_MOVING_STATUS_THRESHOLD
                and abs(m3 - angle_array[2]) < AX_MOVING_STATUS_THRESHOLD
                and abs(m4 - angle_array[3]) < AX_MOVING_STATUS_THRESHOLD
                ):
                    print("check")
                    break
            # next_time = prev_time + (i+1) * (1/FPS)
            
            # time.sleep(max(0,next_time-time.time()))


def main():
    rate = rospy.Rate(10)
    T_motor1, T_motor2, T_motor3 = calc_torque()
    if MISSION == "CIRCLE":
        joint_space, catesian_space, omega = tp_m1(start_angle=[0,0,0,0,0,0], fps=PEN_FPS, center=[8.5+6,-2.17], radius=4.3, c_second=15, start=[7.5+6, 4.9], end=[11+6, -3.5], l_second=1 , z_pos=0.0, mode = 0, s_second=1, e_second=1, b_second=2)
    elif MISSION == "CUBE":
        joint_space, catesian_space, omega = tp_m2(start_angle=[0,0,0,0,0,0], xx=46, aa=6.5, yy=55, bb=4, fps=CUBE_FPS, z= 15, r_second=1, u_second=1, m_second=1, d_second=1)
    AO = Angle_Controller(portHandler, joint_space, omega, T_motor1, T_motor2, T_motor3)

    # 초기 각도, 초당 명령 개수, 원 중심점, 반지름, 원 그리는 시간, 직선 시작점, 끝점, 직선 그리는 시간, 그리퍼 높이
    # mode 0: 둘 다 그리기. mode 1: 원만 그리기, mode 2: 선만 그리기, 그리기 시작점까지 이동시간, 복귀 시간, 브릿지 시간
    print("Bumblebee activated")
    rospy.sleep(2)
    while not rospy.is_shutdown():

        if MISSION == "CIRCLE":
            AO.pen_tracking()
        elif MISSION == "CUBE":
            AO.cube_tracking()
        # AO.AX1_controller.move(512)        # Move AX motor
        # AX2_controller.move(512)        # Move AX motor
        # AX3_controller.move(512)        # Move AX motor
        #AX4_controller.move(512)        # Move AX motor
        #XC1_controller.move(2750)       # 무조건 1850 ~ 2700
        
        rate.sleep()

    portHandler.closePort()


if __name__ == '__main__':
    rospy.init_node("bumblebee", anonymous=True)
    main()

"""
[[1.     0.     0.     0.1305]
 [0.     1.     0.     0.043 ]
 [0.     0.     1.     0.177 ]
 [0.     0.     0.     1.    ]]
"""