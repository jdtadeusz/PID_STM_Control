import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re

PORT = '/dev/ttyACM0'  
BAUD = 115200
WINDOW = 200  

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except serial.SerialException as e:
    print(f"ERROR: Port {PORT} is busy! Close the terminal monitor in VS Code.\n{e}")
    exit()

# Initialize with concrete background data instead of NaN.
# This ensures immediate rendering of a continuous line (oscilloscope behavior).
dist_data  = deque([270]*WINDOW, maxlen=WINDOW)
pwm_data   = deque([1270]*WINDOW, maxlen=WINDOW) # Value close to the baseline
corr_data  = deque([0]*WINDOW, maxlen=WINDOW)
sp_data    = deque([170]*WINDOW, maxlen=WINDOW)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7))
fig.canvas.manager.set_window_title('PID Hovercraft Telemetry')

line_dist, = ax1.plot(dist_data, label='Dist [mm]', color='royalblue', linewidth=1.5)
line_sp,   = ax1.plot(sp_data,   label='Setpoint',  color='red', linestyle='--', linewidth=1)

ax1.set_ylabel('Distance [mm]')
ax1.set_ylim(30, 280)
ax1.set_xlim(0, WINDOW)
ax1.invert_yaxis()  # Sensor is at the top — small value = high altitude
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

line_pwm,  = ax2.plot(pwm_data,  label='PWM', color='orange', linewidth=1.5)
line_corr, = ax2.plot(corr_data, label='PID Correction', color='green', linewidth=1)
ax2.axhline(y=1270, color='gray', linestyle=':', label='Feedforward base')

ax2.set_ylabel('PWM')
ax2.set_ylim(1100, 1600)
ax2.set_xlim(0, WINDOW)
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)
ax2.set_xlabel(f'Samples (last {WINDOW} × 50ms)')

def animate(_):
    current_dist = None
    current_pwm = None
    
    while ser.in_waiting:
        try:
            # errors='ignore' prevents crashes caused by transient garbage bytes on UART
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            # Format: SP:170, Dist:207, Corr:28, PWM:1321, dt:50 ms
            m = re.search(r'SP:(-?\d+),\s*Dist:(-?\d+),\s*Corr:(-?\d+),\s*PWM:(-?\d+)', line)
            
            if m:
                sp, dist, corr, pwm = map(int, m.groups())
                sp_data.append(sp)
                dist_data.append(dist)
                corr_data.append(corr)
                pwm_data.append(pwm)
                
                # Store the latest valid measurements from this batch to update titles
                current_dist = dist
                current_pwm = pwm
        except Exception:
            pass

    # Rebuild the X-axis data array to preserve the scrolling frame effect
    x_data = list(range(WINDOW))
    line_dist.set_data(x_data, list(dist_data))
    line_sp.set_data(x_data, list(sp_data))
    line_pwm.set_data(x_data, list(pwm_data))
    line_corr.set_data(x_data, list(corr_data))
    
    # 2. Display live, real-time measurements directly inside the plot titles
    if current_dist is not None:
        ax1.set_title(f'Ball Position | CURRENT MEASUREMENT: {current_dist} mm', fontweight='bold', color='royalblue')
        ax2.set_title(f'Control Signal | CURRENT PWM: {current_pwm}', fontweight='bold', color='orange')
        
    return line_dist, line_sp, line_pwm, line_corr

plt.tight_layout()
ani = animation.FuncAnimation(fig, animate, interval=50, blit=False, cache_frame_data=False)
plt.show()
ser.close()