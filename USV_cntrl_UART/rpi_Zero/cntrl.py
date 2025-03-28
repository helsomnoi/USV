import serial
import json
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio
import os

class EngServ:
    def __init__(self):
        # Настройка PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        
        # Настройка сервоприводов
        self.servos = [
            servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500),
            servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500),
        ]
        
        # Каналы двигателей
        self.motor_channels = [4, 8]

    def set_motor_speed(self, channel, speed):
        if speed < 0 or speed > 100:
            raise ValueError("Скорость должна быть в диапазоне от 0 до 100")
        pulse_width_ms = 1.0 + (speed / 100) * 1.0
        duty_cycle = int((pulse_width_ms / 20) * 65535)
        self.pca.channels[channel].duty_cycle = duty_cycle

    def set_rudder(self, num, rudder_angle):
        angle = 145 + rudder_angle
        self.servos[num].angle = angle

    def apply_controls(self, u_actual, motor_speed):
        self.set_motor_speed(4, motor_speed)
        self.set_motor_speed(8, motor_speed)
        self.set_rudder(0, u_actual)
        self.set_rudder(1, u_actual)


class RPIReceiver:
    def __init__(self, eng_serv, port="/dev/serial0", baudrate=115200):
        self.eng_serv = eng_serv
        self.port = port
        self.baudrate = baudrate

        if not os.path.exists(port):
            raise FileNotFoundError(f"Порт {port} не найден.")
        
        self.ser = serial.Serial(self.port, self.baudrate)

    def listen(self):
        print(f"Сервер запущен. Ожидание данных по UART на порту {self.port}")
        try:
            while True:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode("utf-8").strip()
                    print(f"Получены данные: {data}")
                    try:
                        received_data = json.loads(data)
                        u_actual = float(received_data.get("u_actual", 0))
                        motor_speed = float(received_data.get("motor_speed", 0))
                        motor_speed = max(0, 0.9*min(100, 100 * motor_speed / 1500))
                        self.eng_serv.apply_controls(u_actual, motor_speed)
                    except (json.JSONDecodeError, ValueError) as e:
                        print(f"Ошибка обработки данных: {e}")
        finally:
            self.close()

    def close(self):
        self.ser.close()
        print("UART закрыт.")


if __name__ == "__main__":
    eng_serv = EngServ()
    receiver = RPIReceiver(eng_serv)
    receiver.listen()

