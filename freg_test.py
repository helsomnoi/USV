import matplotlib.pyplot as plt
from vehicles import *
from lib import *

import numpy as np
import matplotlib.pyplot as plt

# Создание объекта класса frigate
frigate_sim = frigate(U=3.5,r=0)  

# Параметры симуляции
sample_time = 0.01  # Шаг времени (с)
total_time = 35    # Общее время симуляции (с)
time_steps = int(total_time / sample_time)

# Начальное состояние
eta = np.array([0, 0, 0, 0, 0, 0], float)  # Положение [x, y, z, roll, pitch, yaw]
nu = frigate_sim.nu  # Начальная скорость
u_actual = frigate_sim.u_actual  # Управление

# Параметры маневра зиг-заг (длительности меняются)
zigzag_durations = [5, 10, 10, 10]  # В секундах
zig_angle = 20 * (np.pi / 180)  # Угол отклонения руля (в радианах)
zag_angle = -20 * (np.pi / 180) # Угол отклонения руля в противоположную сторону

# Логирование для построения графиков
x_positions = []
y_positions = []
yaw_angles = []
rudder_angles = []

# Переменные для управления маневром
current_duration_index = 0
current_duration = zigzag_durations[current_duration_index]
is_zig = True
elapsed_time_in_phase = 0  # Время, прошедшее с начала текущей фазы

# Запуск симуляции
for t in range(time_steps):
    time = t * sample_time
    elapsed_time_in_phase += sample_time

    # Проверка на завершение текущей фазы и переход к следующей
    if elapsed_time_in_phase >= current_duration:
        elapsed_time_in_phase = 0
        is_zig = not is_zig  # Переключение между зигом и загом
        current_duration_index += 1
        if current_duration_index < len(zigzag_durations):
            current_duration = zigzag_durations[current_duration_index]
        else:
            current_duration = zigzag_durations[-1]  # Последний цикл фиксированной длины

    # Задание управляющего угла руля
    delta_c = zig_angle if is_zig else zag_angle
    u_control = np.array([delta_c], float)
    
    # Обновление состояния судна
    nu, u_actual = frigate_sim.dynamics(eta, nu, u_actual, u_control, sample_time)
    
    # Обновление положения судна
    yaw_angle = eta[5] + nu[5] * sample_time
    eta[5] = yaw_angle  # yaw (psi)
    eta[0] += nu[0] * np.cos(yaw_angle) * sample_time  # x
    eta[1] += nu[0] * np.sin(yaw_angle) * sample_time  # y

    # Сохранение данных для графиков
    x_positions.append(eta[0])
    y_positions.append(eta[1])
    yaw_angles.append(yaw_angle * 180 / np.pi)  # В градусах
    rudder_angles.append(u_actual[0] * 180 / np.pi)  # В градусах

# Построение графика углов курса и руля
plt.figure(figsize=(10, 6))
plt.plot(np.linspace(0, total_time, time_steps), yaw_angles, label="Угол курса (ψ)", color="blue")
plt.plot(np.linspace(0, total_time, time_steps), rudder_angles, label="Угол руля (δ)", color="orange")
plt.xlabel("Время (с)")
plt.ylabel("Угол (градусы)")
plt.title("Зиг-заг тест")
plt.axhline(0, color="black", linestyle="--", linewidth=0.8)
plt.legend()
plt.grid()
plt.show()
