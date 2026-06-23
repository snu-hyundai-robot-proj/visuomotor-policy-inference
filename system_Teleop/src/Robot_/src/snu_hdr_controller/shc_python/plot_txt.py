import sys
import numpy as np
import matplotlib.pyplot as plt

# 사용법: python plot_txt.py data.txt
if len(sys.argv) < 2:
    print("Usage: python plot_txt.py data.txt")
    sys.exit(1)

filename = sys.argv[1]

# N x 12 데이터 로드
data = np.loadtxt(filename)

print("Data shape:", data.shape)  # 확인용 출력

# 12개 컬럼 모두 plot
for i in range(data.shape[1]):
    plt.plot(data[:, i], label=f'col_{i}')

plt.xlabel("Sample index")
plt.ylabel("Value")
plt.title("Nx12 Data Plot")
plt.legend()
plt.grid(True)

plt.show()