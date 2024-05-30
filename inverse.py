from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import matplotlib.pyplot as plt


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
      origin_translation=[0, 4, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    )
])

#목표 위치와 방향
target_position = [26, 4, 11.3]
target_orientation = [[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]] # 3x3 행렬

#역기구학 계산
inv = arm_chain.inverse_kinematics(target_position, target_orientation, orientation_mode='all')

#디버깅을 위한 역산
forw = arm_chain.forward_kinematics(inv)

print(target_position)
print(forw)

#시각화
ax = plt.figure().add_subplot(111, projection='3d')
ax.set_xlabel('x')
ax.set_xlim(0, 30)
ax.set_ylabel('y')
ax.set_ylim(-30, 30)
ax.set_zlabel('z')
ax.set_zlim(0, 30)
arm_chain.plot(inv, ax)
plt.show()