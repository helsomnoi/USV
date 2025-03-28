import matplotlib.pyplot as plt
from vehicles import *
from lib import *
import json
import serial

# Параметры симуляции
sample_time = 0.02  # Шаг времени (с)
total_time = 30    # Общее время симуляции (с)
time_steps = int(total_time / sample_time)

# Начальное состояние
USV_sim = USV(U=3.5)
eta = np.array([0, 0, 0, 0, 0, 0], float)  # Положение [x, y, z, roll, pitch, yaw]
nu = USV_sim.nu  # Начальная скорость
u_actual = USV_sim.u_actual  # Управление

# Логирование для построения графиков
x_positions = []
y_positions = []
yaw_angles = []
rudder_angles = []
w = []

# Создание объекта для отправки данных по UART
uart_sender = UART_sender(port="/dev/serial0", baudrate=115200)

# Симуляция
for t in range(time_steps):
    time = t * sample_time

    delta_c = 35 * (np.pi / 180)  # перевод в радианы
    u_control = np.array([delta_c], float)
    
    # Обновление состояния
    nu, u_actual = USV_sim.dynamics(eta, nu, u_actual, u_control, sample_time)
    
    # Обновление углов и позиций
    yaw_angle = eta[5] + nu[5] * sample_time
    eta[5] = yaw_angle  # yaw (psi)
    eta[0] += nu[0] * np.cos(yaw_angle) * sample_time  # x
    eta[1] += nu[0] * np.sin(yaw_angle) * sample_time  # y

    


    # Сохранение данных для графиков
    x_positions.append(eta[0])
    y_positions.append(eta[1])
    yaw_angles.append(yaw_angle * 180 / np.pi)  # в градусы
    rudder_angles.append(u_actual[0] * 180 / np.pi)  # в градусы
    w.append(nu[5] * 180 / np.pi)


try:
    # Process and send data from simData
    process_and_send_turn(rudder_angles, uart_sender)
finally:
    # Close UART connection
    uart_sender.close()
    
# Построение графиков
plt.figure(figsize=(8, 8))

# График 1: Траектория движения
pl1 = plt.subplot2grid((2, 2), (0, 0))
pl1.plot(y_positions, x_positions)
pl1.set_xlabel("y (м)", fontsize=6)
pl1.set_ylabel("x (м)", fontsize=6)
pl1.set_title("Траектория движения МБЭК")
pl1.grid()

# График 2: Курсовой угол (yaw angle)
pl2 = plt.subplot2grid((2, 2), (0, 1))
pl2.plot(np.linspace(0, total_time, time_steps), np.mod(yaw_angles, 360))
pl2.set_xlabel("Время, сек", fontsize=6)
pl2.set_ylabel("град.", fontsize=6)
pl2.set_title("Курсовой угол (градусы)")
pl2.grid()

# График 3: Угол руля (rudder angle)
pl3 = plt.subplot2grid((2, 2), (1, 0))
pl3.plot(np.linspace(0, total_time, time_steps), rudder_angles)
pl3.set_xlabel("Время, сек", fontsize=6)
pl3.set_ylabel("град", fontsize=6)
pl3.set_title("Рулевой угол (градусы)")
pl3.grid()

# График 4: Угловая скорость (angular velocity)
pl4 = plt.subplot2grid((2, 2), (1, 1))
pl4.plot(np.linspace(0, total_time, time_steps), w)
pl4.set_xlabel("Время, сек", fontsize=6)
pl4.set_ylabel("град/сек", fontsize=6)
pl4.set_title("Угловая скорость (град/сек)")
pl4.grid()

plt.tight_layout()  # Для правильного размещения графиков
plt.show()
