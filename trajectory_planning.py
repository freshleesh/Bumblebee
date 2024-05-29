import numpy as np
import time

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

start = time.time()
th = tp5(0, 90, 10, 1000)
print(time.time() - start)




