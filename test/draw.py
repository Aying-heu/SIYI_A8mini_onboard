import pandas as pd
import matplotlib.pyplot as plt

# 读取结果
df = pd.read_csv("/home/bsa/SIYI_A8mini_onboard/test/tracking_result.csv")

# 绘图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))

# X轴
ax1.plot(df["time_s"], df["obs_x"], "r.-", label="原始观测X")
ax1.plot(df["time_s"], df["pred_x"], "b*-", label="卡尔曼预测X")
ax1.set_title("X轴位置跟踪")
ax1.set_xlabel("时间(s)")
ax1.set_ylabel("位置(m)")
ax1.legend()
ax1.grid(True)

# Y轴
ax2.plot(df["time_s"], df["obs_y"], "r.-", label="原始观测Y")
ax2.plot(df["time_s"], df["pred_y"], "b*-", label="卡尔曼预测Y")
ax2.set_title("Y轴位置跟踪")
ax2.set_xlabel("时间(s)")
ax2.set_ylabel("位置(m)")
ax2.legend()
ax2.grid(True)

# Z轴
ax3.plot(df["time_s"], df["obs_z"], "r.-", label="原始观测Z")
ax3.plot(df["time_s"], df["pred_z"], "b*-", label="卡尔曼预测Z")
ax3.set_title("Z轴位置跟踪")
ax3.set_xlabel("时间(s)")
ax3.set_ylabel("位置(m)")
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.savefig("tracking_plot.png")
plt.show()