import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# ==========================================
# ГЕОМЕТРИЧЕСКИЕ ПАРАМЕТРЫ РУКИ (в мм)
# Замени на свои реальные замеры штангенциркулем/линейкой
# ==========================================
H_BASE = 75.0   # Высота от стола до оси вращения плеча (h)
L1 = 120.0        # Длина плеча (между осями двух MG996R)
L2 = 145.0        # Длина предплечья (от оси локтя до кончика губок)

def servo_to_math(s0, s1, s2):
    # Азимут базы: 90 - строго прямо (0°), 0 - влево (+90°), 180 - вправо (-90°)
    phi = 90.0 - s0
    
    # Угол первого плеча к горизонту (0..140°)
    theta1 = s1 * (140.0 / 180.0)
    
    # Относительный угол в локте: диапазон сжат на 10° с обеих сторон (80° вместо 90°)
    joint2_rel = (90.0 - s2) * (80.0 / 90.0)
    
    # Абсолютный угол наклона второго звена к горизонту (жесткая связка с плечом)
    alpha = theta1 + joint2_rel
    
    return phi, theta1, joint2_rel, alpha

def forward_kinematics(s0, s1, s2):
    phi_deg, theta1_deg, _, alpha_deg = servo_to_math(s0, s1, s2)
    
    phi = np.radians(phi_deg)
    t1 = np.radians(theta1_deg)
    a = np.radians(alpha_deg)
    
    # 0. Центр опоры
    p0 = np.array([0.0, 0.0, 0.0])
    
    # 1. Плечевой сустав
    p1 = np.array([0.0, 0.0, H_BASE])
    
    # 2. Локтевой сустав
    r1 = L1 * np.cos(t1)
    x1 = r1 * np.cos(phi)
    y1 = r1 * np.sin(phi)
    z1 = H_BASE + L1 * np.sin(t1)
    p2 = np.array([x1, y1, z1])
    
    # 3. Конец рабочего органа (TCP)
    r2 = r1 + L2 * np.cos(a)
    x2 = r2 * np.cos(phi)
    y2 = r2 * np.sin(phi)
    z2 = z1 + L2 * np.sin(a)
    p3 = np.array([x2, y2, z2])
    
    return np.array([p0, p1, p2, p3])

# --- Интерфейс ---
fig = plt.figure(figsize=(11, 8.5))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.08, bottom=0.26, right=0.92, top=0.90)

init_s0 = 90.0
init_s1 = 45.0
init_s2 = 90.0
init_s3 = 90.0

pts = forward_kinematics(init_s0, init_s1, init_s2)

arm_line, = ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 
                    '-o', linewidth=6, markersize=8, color='#d35400', 
                    markerfacecolor='#2c3e50', markeredgecolor='black', label="Звенья руки")

tcp_point, = ax.plot([pts[-1, 0]], [pts[-1, 1]], [pts[-1, 2]], 
                     'o', markersize=10, color='#27ae60', label="Захват (TCP)")

max_reach = H_BASE + L1 + L2
ax.set_xlim(-max_reach * 0.8, max_reach * 0.8)
ax.set_ylim(-max_reach * 0.8, max_reach * 0.8)
ax.set_zlim(0, max_reach)

ax.set_xlabel('X (Вперед), мм', fontweight='bold', labelpad=8)
ax.set_ylabel('Y (Влево/Вправо), мм', fontweight='bold', labelpad=8)
ax.set_zlabel('Z (Высота), мм', fontweight='bold', labelpad=8)
ax.set_title('Кинематика 4-DOF манипулятора (Serial Chain)', fontsize=13, fontweight='bold', pad=25)

# Сетка стола
grid_range = np.linspace(-max_reach * 0.8, max_reach * 0.8, 8)
xx, yy = np.meshgrid(grid_range, grid_range)
ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.08, color='black')

ax.legend(loc="upper right", framealpha=0.9, fontsize=9)

info_box = ax.text2D(0.02, 0.98, "", transform=ax.transAxes, fontsize=8.5, verticalalignment='top',
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8f9fa', edgecolor='#bdc3c7', alpha=0.9))

def refresh_display(s0, s1, s2, s3, points):
    phi, t1, j2, a = servo_to_math(s0, s1, s2)
    grip_status = "Сжат" if s3 <= 72 else ("Параллельно" if abs(s3 - 90) <= 2 else "Раскрыт")
    
    info_box.set_text(
        f"Координаты TCP:\n"
        f"  X: {points[-1, 0]:6.1f} мм\n"
        f"  Y: {points[-1, 1]:6.1f} мм\n"
        f"  Z: {points[-1, 2]:6.1f} мм\n"
        f"----------------------\n"
        f"Углы звеньев:\n"
        f"  Азимут базы:    {phi:5.1f}°\n"
        f"  Плечо к столу:  {t1:5.1f}°\n"
        f"  Изгиб в локте:  {j2:5.1f}°\n"
        f"  Локоть к столу: {a:5.1f}°\n"
        f"  Хват: {grip_status} ({s3:.0f}°)\n"
        f"----------------------\n"
        f"Пакет: G {s0:.0f} {s1:.0f} {s2:.0f} {s3:.0f}"
    )

refresh_display(init_s0, init_s1, init_s2, init_s3, pts)

# Слайдеры управления
ax_s0 = plt.axes([0.18, 0.17, 0.55, 0.022])
ax_s1 = plt.axes([0.18, 0.12, 0.55, 0.022])
ax_s2 = plt.axes([0.18, 0.07, 0.55, 0.022])
ax_s3 = plt.axes([0.18, 0.02, 0.55, 0.022])

sl_s0 = Slider(ax_s0, 'Серва 0 (База)', 0.0, 180.0, valinit=init_s0, valstep=1.0)
sl_s1 = Slider(ax_s1, 'Серва 1 (Плечо)', 0.0, 180.0, valinit=init_s1, valstep=1.0)
sl_s2 = Slider(ax_s2, 'Серва 2 (Локоть)', 0.0, 180.0, valinit=init_s2, valstep=1.0)
sl_s3 = Slider(ax_s3, 'Серва 3 (Хват)', 70.0, 120.0, valinit=init_s3, valstep=1.0)

def update(val):
    s0, s1, s2, s3 = sl_s0.val, sl_s1.val, sl_s2.val, sl_s3.val
    p = forward_kinematics(s0, s1, s2)
    
    arm_line.set_data(p[:, 0], p[:, 1])
    arm_line.set_3d_properties(p[:, 2])
    
    tcp_point.set_data([p[-1, 0]], [p[-1, 1]])
    tcp_point.set_3d_properties([p[-1, 2]])
    
    refresh_display(s0, s1, s2, s3, p)
    fig.canvas.draw_idle()

sl_s0.on_changed(update)
sl_s1.on_changed(update)
sl_s2.on_changed(update)
sl_s3.on_changed(update)

# Кнопка сброса
reset_ax = plt.axes([0.80, 0.09, 0.12, 0.045])
btn_reset = Button(reset_ax, 'Нейтраль', color='#bdc3c7', hovercolor='#95a5a6')

def on_reset(event):
    sl_s0.reset()
    sl_s1.reset()
    sl_s2.reset()
    sl_s3.reset()

btn_reset.on_clicked(on_reset)

plt.show()
