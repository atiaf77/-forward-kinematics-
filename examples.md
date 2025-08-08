# نماذج للاستخدام

## مثال 1: الاستخدام الأساسي

```python
import numpy as np
from forward_kinematics import forward_kinematics, extract_position_and_orientation

# تعريف زوايا المفاصل
joint_angles = [0, np.pi/4, np.pi/6, 0, np.pi/3]

# حساب Forward Kinematics
T = forward_kinematics(joint_angles)
position, rotation = extract_position_and_orientation(T)

print(f"الموضع: {position}")
print(f"المسافة من الأصل: {np.linalg.norm(position):.2f} mm")
```

## مثال 2: حساب مسار

```python
import numpy as np
import matplotlib.pyplot as plt

# تعريف مسار للمفصل الأول
theta1_values = np.linspace(0, 2*np.pi, 100)
positions = []

for theta1 in theta1_values:
    joint_angles = [theta1, 0, 0, 0, 0]
    T = forward_kinematics(joint_angles)
    pos, _ = extract_position_and_orientation(T)
    positions.append(pos)

positions = np.array(positions)

# رسم المسار
plt.figure(figsize=(10, 6))
plt.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
plt.xlabel('X (mm)')
plt.ylabel('Y (mm)')
plt.title('مسار End-Effector عند دوران المفصل الأول')
plt.grid(True)
plt.axis('equal')
plt.show()
```

## مثال 3: مقارنة أوضاع مختلفة

```python
configurations = [
    ([0, 0, 0, 0, 0], "الوضع الأساسي"),
    ([np.pi/2, 0, 0, 0, 0], "دوران 90° للقاعدة"),
    ([0, np.pi/2, 0, 0, 0], "رفع الكتف 90°"),
    ([0, 0, np.pi/2, 0, 0], "ثني الكوع 90°")
]

print("مقارنة الأوضاع المختلفة:")
print("-" * 50)

for angles, desc in configurations:
    T = forward_kinematics(angles)
    pos, _ = extract_position_and_orientation(T)
    reach = np.linalg.norm(pos)
    
    print(f"{desc}:")
    print(f"  الموضع: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
    print(f"  المدى: {reach:.1f} mm")
    print()
```
