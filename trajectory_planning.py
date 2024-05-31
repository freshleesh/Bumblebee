import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


def robot_chain():
    #링크의 집합으로 된 chain 생성
    arm_chain = Chain(name='left_arm', links=[
        URDFLink(
        name="link_0",
        #해당 링크의 끝점 (dh파라미터에서 z축 위치) (모터의 시작점)
        origin_translation=[0, 0, 0],
        #그 다음 링크를 얼마나 회전해서 연결할 것인가 (그냥 0 0 0하면 됨)
        origin_orientation=[0, 0, 0],
        #링크의 회전 방향
        rotation=[0, 0, 1],
        ),
        URDFLink(
        name="link_1",
        origin_translation=[0, 0, 11.3],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        ),
        URDFLink(
        name="link_2",
        origin_translation=[0, 0, 13.0],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        ),
        URDFLink(
        name="link_3",
        origin_translation=[0, 0, 13.0],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
        ),
        URDFLink(
        name="link_4",
        origin_translation=[0, 0, 5],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        )
    ])
    return arm_chain





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

def draw_circle(center, radius, num_of_point):
    # 중심 위치 [x, y], 반지름, 점 개수
    
    # 원을 이루는 각 0 ~ 2pi까지 생성
    theta = np.linspace(0, 2*np.pi, num_of_point)

    # theta를 매개변수로 하는 원의 궤도 return  [ [x, x, x], [y, y, y] ]
    circle = np.array([center[0] + radius * np.cos(theta), center[1] + radius * np.sin(theta)])

    circle = circle.transpose()

    return circle


def draw_line(start, end, num_of_point):
    #시작, 끝 점 좌표 [x,y], 점 개수

    # start부터 end까지 linear하게 쪼개기
    x = np.linspace(start[0], end[0], num_of_point) 
    y = np.linspace(start[1], end[1], num_of_point) 

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

    # 결과를 담을 행렬
    destination = []

    #블럭 사이 간격
    delta = 0.4

    # 1층 블럭
    for i in range(4):
        # 블럭 크기 2.5 ^ 3
        x = np.sin(np.deg2rad(180 - yy)) * (bb + 1.5 + delta * (i+1) + 2.5 / 2 + 2.5 * i)
        y = np.cos(np.deg2rad(180 - yy)) * (bb + 1.5 + delta * (i+1) + 2.5 / 2 + 2.5 * i)
        z = 7 + 2.5 / 2 

        destination.append([x, y, z])

    # 2층 블럭
    for i in range(3):
        x = (destination[i][0] + destination[i+1][0])/2
        y = (destination[i][1] + destination[i+1][1])/2
        z = destination[i][2] + 2.5 

        destination.append([x, y, z])

    # 3층 블럭
    for i in range(2):
        x = (destination[i+4][0] + destination[i+5][0])/2
        y = (destination[i+4][1] + destination[i+5][1])/2
        z = destination[i+4][2] + 2.5 

        destination.append([x, y, z])

    # 4층 블럭
    x = (destination[-1][0] + destination[-2][0])/2
    y = (destination[-1][1] + destination[-2][1])/2
    z = destination[-1][2] + 2.5

    destination.append([x, y, z])

    # 넘파이화
    destination = np.array(destination)

    return destination


def tp_m1(start_angle, fps, center, radius, c_second, start, end, l_second, z_pos, mode, s_second, e_second, b_second):
    #초기 각도, 초당 명령 개수, 원 중심점, 반지름, 원 그리는 시간, 직선 시작점, 끝점, 직선 그리는 시간, 그리퍼 높이, mode 0: 둘 다 그리기. mode 1: 원만 그리기, mode 2: 선만 그리기, 그리기 시작점까지 이동시간, 복귀 시간, 브릿지 시간

    # 로봇 모델 생성
    arm_chain = robot_chain()

    target_orientation = [[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]] # 3x3 행렬
    
    joint_traj = []

    # waypoint 생성
    circle = draw_circle(center, radius, fps * c_second)
    line = draw_line(start, end, fps * l_second)

    # z값 추가
    zs = np.ones((fps * c_second, 1)) * z_pos
    circle = np.hstack((circle, zs))
    line = np.hstack((line,  zs))

    # line과 circle사이 잇는 trajectory 반원으로 만들었다.
    b_start = circle[-1]
    b_end = line[0]
    b_center = (b_start + b_end) / 2
    r = b_start - b_center
    r = np.linalg.norm(r)
    th = np.linspace(0, np.pi, fps * b_second)
    z = np.sin(th) * r + z_pos
    th2 = np.arctan2(b_end[1] - b_start[1], b_end[0] - b_start[0])
    x = r * np.cos(th) * np.sin(th2) + b_center[0]
    y = r * np.cos(th) * np.cos(th2) + b_center[1]

    bridge = np.array([x, y, z])
    bridge = bridge.transpose()

    # 모드 별 카테시안 생성
    if mode == 0:
        cartesian_traj = np.vstack((circle, bridge, line))
    elif mode == 1:
        cartesian_traj = circle
    else:
        cartesian_traj = line
    
    # 역기구학 계산
    for target_position in cartesian_traj:
        joint_traj.append(arm_chain.inverse_kinematics(target_position, target_orientation, orientation_mode='all'))

    # start_angle하고 연결
    to_start = tp5(start_angle, joint_traj[0], s_second, fps)
    to_end = tp5(joint_traj[-1], start_angle, e_second, fps)
    joint_traj = np.vstack((to_start, np.array(joint_traj), to_end))

    #시각화를 위한 정기구학
    end_pose = arm_chain.forward_kinematics(start_angle)
    cartesian_traj = end_pose[:, 3]
    for angles in joint_traj:
        end_pose = arm_chain.forward_kinematics(angles)
        cartesian_traj = np.vstack((cartesian_traj, end_pose[:,3]))

    return joint_traj, cartesian_traj

if __name__ == "__main__":
    #예시
    # th = tp5([0, 10], [10, 0], 10, 10)
    # # print(th)
    # circle = draw_circle([5, 5], 1, 1000)
    # line = draw_line([0, 0], [5, 5], 1000)
    # pickup = pickup_location(45, 6.5)
    # destination = place_location(35, 4)
    
    st = time.time()
    drawing_traj, ctraj = tp_m1([0,0,0,0,0], 40, [5,5], [3], 10, [3, 10], [6, -10], 10, 2, 0, 5, 5, 3)
    print('time: ', time.time() - st)

    #시각화
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.set_xlim(0, 30)
    ax.set_xlabel('x')
    ax.set_ylim(-20, 20)
    ax.set_ylabel('y')
    ax.set_zlim(0, 50)

    ax.scatter(ctraj[:,0], ctraj[:,1], ctraj[:,2])
    # ax.scatter(pickup[0], pickup[1], pickup[2])
    # ax.scatter(destination[:,0], destination[:,1], destination[:,2])
    # ax.plot3D(line[:,0], line[:,1], np.zeros(1000))
    # ax.plot3D(circle[:,0], circle[:,1], np.zeros(1000))
    plt.show()








