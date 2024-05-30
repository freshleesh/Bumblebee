import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def tp5(start, end, duration, step):
    # 시작값, 끝값, 작동시간, 궤적 개수

    # 5차항 계수
    c1 = (end - start) / (duration**5/120)

    # 상수항
    c2 = start

    # 완성된 함수
    f = [c1/20, -c1*duration/8, c1*duration*duration/12, 0, 0, c2]

    # 작은 step으로 나누기
    t = np.linspace(0, duration, step)

    # 함수에 대입
    th = np.polyval(f, t)
    return th

def draw_circle(center, radius, num_of_point):
    # 중심 위치 [x, y], 반지름, 점 개수
    
    # 원을 이루는 각 0 ~ 2pi까지 생성
    theta = np.linspace(0, 2*np.pi, num_of_point)

    # theta를 매개변수로 하는 원의 궤도 return  [ [x, x, x], [y, y, y] ]
    circle = np.array([center[0] + radius * np.cos(theta), center[1] + radius * np.sin(theta)])

    return circle


def draw_line(start, end, num_of_point):
    #시작, 끝 점 좌표 [x,y], 점 개수

    # start부터 end까지 linear하게 쪼개기
    x = np.linspace(start[0], end[0], num_of_point) 
    y = np.linspace(start[1], end[1], num_of_point) 

    # return  [ [x, x, x], [y, y, y] ]
    line = np.array([x, y])

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
        x = np.cos(np.deg2rad(180 - yy)) * (bb + 1.5 + delta * (i+1) + 2.5 / 2 + 2.5 * i)
        y = np.sin(np.deg2rad(180 - yy)) * (bb + 1.5 + delta * (i+1) + 2.5 / 2 + 2.5 * i)
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


#예시
th = tp5(0, 90, 10, 1000)
circle = draw_circle([0, 0], 1, 1000)
line = draw_line([-1, -1], [1, 1], 1000)
pickup = pickup_location(45, 6.5)
destination = place_location(35, 4)



#시각화
fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
ax.set_xlim(10, -10)
ax.set_ylim(0, 20)
ax.set_zlim(0, 20)
ax.scatter(pickup[0], pickup[1], pickup[2])
ax.scatter(destination[:,0], destination[:,1], destination[:,2])
plt.show()








