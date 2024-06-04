import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


def gripper_chain():
    # 그리퍼 달린 로봇 체인 생성
    gripper_chain = Chain(name='left_arm', links=[
        URDFLink(
        name="link_0",
        #해당 링크의 끝점 (dh파라미터에서 z축 위치) (모터의 시작점)
        origin_translation=[0, 0, 0],
        #그 다음 링크를 얼마나 회전해서 연결할 것인가 (그냥 0 0 0하면 됨)
        origin_orientation=[0, 0, 0],
        #링크의 회전 방향
        rotation=[0, 0, 1],
        #제약사항
        bounds=(-3.14, 3.14)
        ),
        URDFLink(
        name="link_1",
        origin_translation=[0, 0, 11.3], # 여기를 짧게 하니까 수직으로 잘 감
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        bounds =(-3.14/2, 3.14/2)
        ),
        URDFLink(
        name="link_2",
        origin_translation=[0, 0, 13.0],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        bounds =(-3.14, 0.1)
        ),
        URDFLink(
        name="link_3",
        origin_translation=[0, 0, 13.0],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        bounds =(-3.14, 0.1)
        ),
        URDFLink(
        name="gripper_body",
        origin_translation=[0, 0, 6.7],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        ),
        URDFLink(
        name="gripper",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        )

    ], active_links_mask=[True, True, True, True, False, False])
    return gripper_chain

def pen_chain():
    #링크의 집합으로 된 chain 생성
    pen_chain = Chain(name='left_arm', links=[
        URDFLink(
        name="link_0",
        #해당 링크의 끝점 (dh파라미터에서 z축 위치) (모터의 시작점)
        origin_translation=[0, 0, 0],
        #그 다음 링크를 얼마나 회전해서 연결할 것인가 (그냥 0 0 0하면 됨)
        origin_orientation=[0, 0, 0],
        #링크의 회전 방향
        rotation=[0, 0, 1],
        #제약사항
        bounds=(-3.14, 3.14)
        ),
        URDFLink(
        name="link_1",
        origin_translation=[0, 0, 11.3 - 10],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        bounds =(-3.14, -3.14/2)
        ),
        URDFLink(
        name="link_2",
        origin_translation=[0, 0, 13.0],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        bounds =(-3.14, 0.01)
        ),
        URDFLink(
        name="link_3",
        origin_translation=[0, 0, 13.0],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        bounds =(-3.14, 0.1)
        ),
        URDFLink(
        name="pen_holder",
        origin_translation=[0, 4, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        ),
        URDFLink(
        name="pen",
        origin_translation=[5.5, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        )

    ], active_links_mask=[True, True, True, True, False, False])
    return pen_chain


def tp5(start, end, duration, fps):
    # 시작값, 끝값, 작동시간, 초당 프레임

    start = np.array(start)
    end = np.array(end)

    # 5차항 계수 coefficient 5 s
    c5s = (end - start) / (duration**5/120)

    # 상수항
    c0s = start

    # 작은 step으로 나누기
    t = np.linspace(0, duration, fps * duration)

    angles = []

    # 각 모터별로
    for c5, c0 in zip(c5s, c0s):
        # 다항식 만들어서
        f = np.array([c5/20, -c5*duration/8, c5*duration*duration/12, 0, 0, c0])
        # 계산
        angles.append(np.polyval(f, t))

    angles = np.array(angles)

    angles = angles.transpose()
    
    return angles

def tp5_single(start, end, duration, fps):
    c5 = (end - start) / (duration**5/120)
    c0 = start
    t = np.linspace(0, duration, fps * duration)
    f = np.array([c5/20, -c5*duration/8, c5*duration*duration/12, 0, 0, c0])
    angles = np.polyval(f, t)

    return angles

def draw_circle(center, radius, fps, duration):
    # 중심 위치 [x, y], 반지름, 점 개수
    
    # 원을 이루는 각 0 ~ 2pi까지 생성
    theta = tp5_single(0, 2.1*np.pi, duration, fps)
    # theta = np.linspace(0, 2*np.pi, num_of_point)

    # theta를 매개변수로 하는 원의 궤도 return  [ [x, x, x], [y, y, y] ]
    circle = np.array([center[0] + radius * np.cos(theta), center[1] + radius * np.sin(theta)])

    circle = circle.transpose()

    return circle


def draw_line(start, end, fps, duration):
    #시작, 끝 점 좌표 [x,y], 점 개수

    # start부터 end까지 linear하게 쪼개기
    x = tp5_single(start[0], end[0], duration, fps) 
    y = tp5_single(start[1], end[1], duration, fps) 

    # return  [ [x, x, x], [y, y, y] ]
    line = np.array([x, y])

    line = line.transpose()

    return line

def pickup_location(xx, aa):
    # xx: deg, aa: cm

    # 삼각함수로 pick up 물체 정 중앙 위치
    x = np.cos(np.deg2rad(xx)) * (aa + 3)
    y = np.sin(np.deg2rad(xx)) * (aa + 3)
    
    # 받침대 높이 + 큐브 높이 절반
    z = 7 + 2.5 / 2

    return [x, y, z]


def place_location(yy, bb):
    # yy: deg, bb: cm     
    # 베이스 높이 7cm
    # 큐브 정 중앙 위치

    #블럭 사이 간격
    delta = 0.4

    # 1층 블럭
    first_floor = []
    for i in range(4):
        # 블럭 크기 2.5 ^ 3
        x = np.sin(np.deg2rad(180 - yy)) * (bb + 1.5 + delta * (i+1) + 2.5 / 2 + 2.5 * i)
        y = np.cos(np.deg2rad(180 - yy)) * (bb + 1.5 + delta * (i+1) + 2.5 / 2 + 2.5 * i)
        z = 7 + 2.5 / 2 

        first_floor.append([x, y, z])
    first_floor.reverse()

    destination = first_floor

    # 2층 블럭
    second_floor = []
    for i in range(3):
        x = (first_floor[i][0] + first_floor[i+1][0])/2
        y = (first_floor[i][1] + first_floor[i+1][1])/2
        z = first_floor[i][2] + 2.5 

        second_floor.append([x, y, z])
        # second_floor.reverse()

    destination += second_floor

    # 3층 블럭
    third_floor = []
    for i in range(2):
        x = (second_floor[i][0] + second_floor[i+1][0])/2
        y = (second_floor[i][1] + second_floor[i+1][1])/2
        z = second_floor[i][2] + 2.5 

        third_floor.append([x, y, z])
        # third_floor.reverse()

    destination += third_floor

    # 4층 블럭
    x = (third_floor[-1][0] + third_floor[-2][0])/2
    y = (third_floor[-1][1] + third_floor[-2][1])/2
    z = third_floor[-1][2] + 2.5

    destination.append([x, y, z])

    # 넘파이화
    destination = np.array(destination)

    return destination


def tp_m2(start_angle, xx, aa, yy, bb, fps, z, r_second, u_second, m_second, d_second):
    # 초기 각도, pick deg, pick dis, place deg, place dis, fps, midpoint z, ready, up, mid, down time
    gripper_robot = gripper_chain()
    target_orientation = [0, 0 ,-1]

    # way point 생성
    pickup = pickup_location(xx, aa)
    place = place_location(yy, bb)

    # pick할 곳 공중 waypoint
    up_pos = np.array([pickup[0], pickup[1], z])
    
    # place할 곳 공중 waypoints
    down_poses = np.array([place[:,0], place[:,1], [z]*len(place)])
    down_poses = down_poses.transpose()

    # cartesian으로 planning할 부분 잇기    
    pickup_up_ct = tp5(pickup, up_pos, u_second, fps)
    up_pickup_ct = np.flip(pickup_up_ct, axis=0)

    # down_box 는 3차원임 box가 여러개라
    down_box_ct = []
    for down, box in zip(down_poses, place):
        down_box_ct.append(tp5(down, box, d_second, fps))
    down_box_ct = np.array(down_box_ct)

    # way point들 joint space로 변환
    # start - up
    up_joint = gripper_robot.inverse_kinematics(up_pos, target_orientation, orientation_mode="Z")
    start_up_joint = tp5(start_angle, up_joint, r_second, fps)
    up_start_joint = np.flip(start_up_joint, axis=0)

    # up - pickup
    up_pickup_joint = []
    for cat in up_pickup_ct:
        up_pickup_joint.append(gripper_robot.inverse_kinematics(cat, target_orientation, 'Z'))
    up_pickup_joint = np.array(up_pickup_joint)
    pickup_up_joint = np.flip(up_pickup_joint, axis=0)

    # down 
    down_joint = []
    for down in down_poses:
        down_joint.append(gripper_robot.inverse_kinematics(down, target_orientation, 'Z'))
    down_joint = np.array(down_joint)

    # up - down 3차원
    up_down_joint = []
    for down in down_joint:
        up_down_joint.append(tp5(up_joint, down, m_second, fps))
    up_down_joint = np.array(up_down_joint)
    down_up_joint = np.flip(up_down_joint, axis=1)

    # down - box 3차원
    down_box_joint = []
    for down_box in down_box_ct:
        down_box_single_jt = []
        for down_box_waypoint in down_box:
            down_box_single_jt.append(gripper_robot.inverse_kinematics(down_box_waypoint, target_orientation, 'Z'))
        down_box_joint.append(down_box_single_jt)
    down_box_joint = np.array(down_box_joint)
    box_down_joint = np.flip(down_box_joint, axis=1)

    # joint space point 잇기
    joint_traj = start_up_joint
    for box_num, down_box_single_jt in enumerate(down_box_joint):
        joint_traj = np.vstack((joint_traj, up_pickup_joint, np.array([100, 0, 0, 0, 0, 0]), pickup_up_joint, up_down_joint[box_num], down_box_single_jt, np.array([-100, 0, 0, 0, 0, 0]), box_down_joint[box_num], down_up_joint[box_num]))
    joint_traj = np.vstack((joint_traj, up_start_joint))

    cartesian_traj = np.zeros(1)

    # 각속도 변환
    omega = np.diff(joint_traj) * fps
    omega = np.vstack((omega[0], omega))

    return joint_traj, cartesian_traj, omega


def tp_m1(start_angle, fps, center, radius, c_second, start, end, l_second, z_pos, mode, s_second, e_second, b_second):
    #초기 각도, 초당 명령 개수, 원 중심점, 반지름, 원 그리는 시간, 직선 시작점, 끝점, 직선 그리는 시간, 그리퍼 높이, mode 0: 둘 다 그리기. mode 1: 원만 그리기, mode 2: 선만 그리기, 그리기 시작점까지 이동시간, 복귀 시간, 브릿지 시간

    # 로봇 모델 생성
    pen_robot = pen_chain()

    target_orientation = [[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, -1]] # 3x3 행렬
    
    joint_traj = []

    # waypoint 생성
    circle = draw_circle(center, radius, fps, c_second)
    line = draw_line(start, end, fps, l_second)

    # z값 추가
    zs1 = np.ones((fps * c_second, 1)) * z_pos
    circle = np.hstack((circle, zs1))
    zs2 = np.ones((fps * l_second, 1)) * z_pos
    line = np.hstack((line,  zs2))


    # =====================직사각형=======================

    circle_above = np.array([circle[-1,0],circle[-1,1],circle[-1,2] + 3])
    # 끝날떄도
    line_above = np.array([line[0,0],line[0,1],line[0,2] + 3])

    # 수직점 직선으로 연결
    circle_high = tp5(circle[-1], circle_above, 1, fps)
    high_high = tp5(circle_above, line_above, 1, fps)
    line_hight = tp5(line_above, line[0], 1, fps)

    bridge = np.vstack((circle_high, high_high, line_hight))

    # ======================반원======================
    # line과 circle사이 잇는 trajectory 반원으로 만들었다.
    # b_start = circle[0]
    # b_end = line[0]
    # b_center = (b_start + b_end) / 2.0
    # r = b_start - b_center
    # r = np.linalg.norm(r)
    # th = tp5_single(0, np.pi, b_second, fps)
    # z = np.sin(th) * r *2  + z_pos
    # x = tp5_single(b_start[0], b_end[0], b_second, fps)
    # y = tp5_single(b_start[1], b_end[1], b_second, fps)
    # print(x,y,z)
    # bridge = np.array([x, y, z])
    # bridge = bridge.transpose()

    # 모드 별 카테시안 생성
    if mode == 0:
        cartesian_traj = np.vstack((circle, bridge, line))
    elif mode == 1:
        cartesian_traj = circle
    else:
        cartesian_traj = line

    # 그리기 시작할때 수직으로 접근하기
    start_above = np.array([cartesian_traj[0,0],cartesian_traj[0,1],cartesian_traj[0,2] + 3])
    # 끝날떄도
    end_above = np.array([cartesian_traj[-1,0],cartesian_traj[-1,1],cartesian_traj[-1,2] + 3])

    # 수직점 직선으로 연결
    start_pos = tp5(start_above, cartesian_traj[0], 1, fps)
    end_pos = tp5(cartesian_traj[-1], end_above, 1, fps)

    
    cartesian_traj = np.vstack((start_pos, cartesian_traj, end_pos))


    # 역기구학 계산
    for target_position in cartesian_traj:
        joint_traj.append(pen_robot.inverse_kinematics(target_position, target_orientation, orientation_mode='all'))

    # start_angle하고 연결
    to_start = tp5(start_angle, joint_traj[0], s_second, fps)
    to_end = tp5(joint_traj[-1], start_angle, e_second, fps)
    joint_traj = np.vstack((to_start, np.array(joint_traj), to_end))

    # 각속도 값 생성
    omega = np.diff(joint_traj) * fps

    omega = np.vstack((omega[0], omega))
    

   

    return joint_traj, cartesian_traj, omega

if __name__ == "__main__":
    
    fps = 10
    st = time.time()
    

    drawing_traj, ctraj, omega = tp_m2(start_angle=[0,0,0,0,0,0], xx=45, aa=15, yy=35, bb=10, fps=fps, z= 20, r_second=3, u_second=1, m_second=1, d_second=1) 
    place = place_location(35,10)
    pick = np.array(pickup_location(45,15))
    # drawing_traj, ctraj, omega = tp_m1(start_angle=[0,0,0,0,0,0], fps=fps, center=[8+6,-2], radius=4, c_second=5, start=[7+6, 5], end=[11.25+6, -3.75], l_second=1, z_pos=0, mode = 0, s_second=3, e_second=3, b_second=1)
    
    print('planning time: ', time.time() - st)
    robot = gripper_chain()
    # robot = pen_chain()
    


    #시각화
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.set_xlim(0, 30)
    ax.set_xlabel('x')
    ax.set_ylim(-20, 20)
    ax.set_ylabel('y')
    ax.set_zlim(0, 50)


    start_time = time.time()
    points = []
    for i, angles in enumerate(drawing_traj):

        # 잡을 시간
        if angles[0] > 50:
            print("grip")

        # 놓을 시간
        elif angles[0] < -50:
            print('drop')

        else:
            ax.clear()
            ax.set_xlim(0, 40)
            ax.set_xlabel('x')
            ax.set_ylim(-20, 20)
            ax.set_ylabel('y')
            ax.set_zlim(0, 40)
            robot.plot(angles, ax)
            pose = robot.forward_kinematics(angles)
            points.append(pose[:,3])
            points = np.array(points)
            ax.plot3D(points[:,0] , points[:,1] , points[:,2] )
            points = list(points)

            # =====box, pick====
            ax.scatter(place[:,0], place[:,1], place[:,2])
            ax.scatter(pick[0], pick[1], pick[2])


            #다음 명령 시간 될때까지 대기
            next_time = start_time + (i + 1) * (1 / fps)
            time.sleep(max(0, next_time - time.time()))

            plt.pause(1/fps)
    print("operating time:", time.time() - start_time)
    print("frame time: ",(time.time() - start_time)/fps)

    plt.show()








