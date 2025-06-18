import time
import os
import numpy as np
import matplotlib.pyplot as plt

# ------ 설정 ------
log_file = "/dev/shm/servo_intervals.csv"
DOF = 6  # 로봇 자유도 (6으로 가정)
max_history = 200   # pos/vel/acc 시계열 길이
interval_window = 20  # interval min/max/avg 계산 구간(슬라이딩)

intervals = []
pos_hist = []
vel_hist = []
acc_hist = []
last_line = None

plt.ion()
fig, axs = plt.subplots(4, 1, figsize=(12, 10), gridspec_kw={'height_ratios':[2, 1, 1, 1]})
ax_interval, ax_pos, ax_vel, ax_acc = axs

# pos/vel/acc plot 준비
lines_pos = [ax_pos.plot([], [], label=f'pos{i+1}')[0] for i in range(DOF)]
lines_vel = [ax_vel.plot([], [], label=f'vel{i+1}')[0] for i in range(DOF)]
lines_acc = [ax_acc.plot([], [], label=f'acc{i+1}')[0] for i in range(DOF)]
for ax, ylab in zip([ax_pos, ax_vel, ax_acc], ['Position(joint)', 'Velocity', 'Acceleration']):
    ax.set_ylabel(ylab)
    ax.legend(loc='upper right')
ax_acc.set_xlabel("realtime control monitoring plot")

while True:
    try:
        with open(log_file, "r") as f:
            line = f.readline().strip()
            if not line or line == last_line:
                continue
            last_line = line

            items = line.split(",")
            if len(items) < 1 + 1 + 3 * DOF:
                continue
            interval = float(items[1])
            pos = []
            vel = []
            acc = []
            for i in range(DOF):
                base = 2 + i*3
                pos.append(float(items[base]))
                vel.append(float(items[base+1]))
                acc.append(float(items[base+2]))

            intervals.append(interval)
            pos_hist.append(pos)
            vel_hist.append(vel)
            acc_hist.append(acc)
            # 오래된 것 자르기
            if len(intervals) > max_history:
                intervals = intervals[-max_history:]
                pos_hist = pos_hist[-max_history:]
                vel_hist = vel_hist[-max_history:]
                acc_hist = acc_hist[-max_history:]

            # --- 1. Interval Min/Max/Avg Plot (Top subplot) ---
            if intervals:
                min_val = np.min(intervals)
                max_val = np.max(intervals)
                avg_val = np.mean(intervals)
                x = [0]  # x축 중앙에 한 번만

                ax_interval.clear()
                ax_interval.set_title("Interval (ms) - Min/Max/Average (Current)")
                # min/max: 검정 점, avg: 회색 네모
                ax_interval.scatter(x, [min_val], color='black', label='Min/Max', zorder=3, s=100)
                ax_interval.scatter(x, [max_val], color='black', zorder=3, s=100)
                ax_interval.scatter(x, [avg_val], color='red', marker='s', label='Average', zorder=3, s=100)
                ax_interval.plot([x[0], x[0]], [min_val, max_val], color='black', lw=5, zorder=1)
                ax_interval.set_ylabel("Interval (ms)")
                ax_interval.legend(loc='upper right')
                ax_interval.set_ylim(1.8, max(max_val * 1.1, 2.2))
                ax_interval.set_xlim(-1, 1)
                ax_interval.grid(True, linestyle='--', alpha=0.3)
                ax_interval.set_xticks([])
                # ★ 평균값 점 오른쪽에 수치 표시 (조금 오른쪽, 수직 중앙 정렬)
                ax_interval.text(
                    x[0] + 0.1, avg_val,  # x 위치는 점보다 약간 오른쪽
                    f"average : {avg_val:.4f} ms",     # 소수점 4자리까지
                    va='center', ha='left', color='red', fontsize=12, weight='bold'
                )
               
            # --- 2. pos/vel/acc Timeseries ---
            xs = range(len(pos_hist))
            for i in range(DOF):
                lines_pos[i].set_xdata(xs)
                lines_pos[i].set_ydata([h[i] for h in pos_hist])
                lines_vel[i].set_xdata(xs)
                lines_vel[i].set_ydata([h[i] for h in vel_hist])
                lines_acc[i].set_xdata(xs)
                lines_acc[i].set_ydata([h[i] for h in acc_hist])

            for ax in [ax_pos, ax_vel, ax_acc]:
                ax.relim()
                ax.autoscale_view()
                ax.grid(True, linestyle='--', alpha=0.3)

            plt.tight_layout()
            plt.pause(0.01)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"Error: {e}")
    time.sleep(0.01)
