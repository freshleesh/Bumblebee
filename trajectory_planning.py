import numpy as np
import time
import matplotlib.pyplot as plt

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

    x = np.cos(np.deg2rad(xx)) * aa
    y = np.sin(np.deg2rad(xx)) * aa
th = tp5(0, 90, 10, 1000)
circle = draw_circle([0, 0], 1, 1000)
line = draw_line([-1, -1], [1, 1], 1000)
plt.plot(circle[0], circle[1], line[0], line[1])
plt.show()








