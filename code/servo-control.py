import time
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk

# Поиск порта ESP32
port_name = None
for p in serial.tools.list_ports.comports():
    desc = p.description.lower()
    if any(k in desc for k in ["cp210", "ch340", "usb serial", "uart", "esp"]):
        port_name = p.device
        break

if not port_name:
    # Запасной вариант для Windows (укажи свой COM-порт при необходимости)
    port_name = "COM3"

print(f"Подключение к: {port_name}")

try:
    ser = serial.Serial(port_name, 115200, timeout=0.1)
    time.sleep(2)  # ожидание перезагрузки ESP32 при открытии порта
    print("Порт успешно открыт!")
except Exception as e:
    print(f"Не удалось открыть порт: {e}")
    ser = None

def send_angles():
    # Проверяем, что созданы ровно 4 слайдера
    if len(sliders) < 4:
        return

    if ser and ser.is_open:
        angles = [s.get() for s in sliders]
        cmd = f"G {angles[0]:.1f} {angles[1]:.1f} {angles[2]:.1f} {angles[3]:.1f}\n"
        ser.write(cmd.encode("ascii"))
        ser.readline()

def on_slider_move(_=None):
    send_angles()

def set_home():
    for s in sliders:
        s.set(90)
    send_angles()

def stop_servos():
    if ser and ser.is_open:
        ser.write(b"STOP\n")

# Графическое окно
root = tk.Tk()
root.title("Управление манипулятором")
root.geometry("380x360")

joint_names = [
    "0: База (MG996R)",
    "1: Плечо (MG996R)",
    "2: Локоть (MG996R)",
    "3: Захват (MG90S)"
]

sliders = []

# 1. Создаем виджеты и задаем начальное значение 90° БЕЗ привязки колбэка
for name in joint_names:
    frame = ttk.Frame(root, padding=6)
    frame.pack(fill="x")
    ttk.Label(frame, text=name, font=("Arial", 9, "bold")).pack(anchor="w")
    
    s = ttk.Scale(frame, from_=0, to=180, orient="horizontal")
    s.set(90)
    s.pack(fill="x")
    sliders.append(s)

# 2. Только когда ВСЕ 4 слайдера созданы, подключаем обработчик движения
for s in sliders:
    s.configure(command=on_slider_move)

btn_frame = ttk.Frame(root, padding=10)
btn_frame.pack(fill="x", pady=10)

ttk.Button(btn_frame, text="Домой (90°)", command=set_home).pack(side="left", expand=True, padx=4)
ttk.Button(btn_frame, text="STOP (Расслабить)", command=stop_servos).pack(side="right", expand=True, padx=4)

# Отправляем начальное нейтральное положение в ESP32
send_angles()

root.mainloop()

if ser and ser.is_open:
    ser.close()
