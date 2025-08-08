import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_robot_arm(DH_params, joint_angles, title="Robot Arm Configuration"):
    """
    رسم الذراع الروبوتية في الفضاء ثلاثي الأبعاد
    
    Parameters:
    DH_params: معاملات DH
    joint_angles: زوايا المفاصل
    title: عنوان الرسم
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # حساب موضع كل مفصل
    positions = []
    T = np.eye(4)
    positions.append(T[:3, 3])  # الموضع الأساسي
    
    for i in range(len(DH_params)):
        a, alpha, d, theta_offset = DH_params[i]
        theta_total = theta_offset + joint_angles[i]
        T_i = dh_transform(a, alpha, d, theta_total)
        T = np.dot(T, T_i)
        positions.append(T[:3, 3])
    
    positions = np.array(positions)
    
    # رسم الروابط
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=3, label='Robot Links')
    
    # رسم المفاصل
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
               c='red', s=100, label='Joints')
    
    # رسم الأرقام على المفاصل
    for i, pos in enumerate(positions):
        ax.text(pos[0], pos[1], pos[2], f'J{i}', fontsize=12)
    
    # رسم نظام الإحداثيات للـ end-effector
    end_pos = positions[-1]
    T_final = forward_kinematics(joint_angles)
    rotation_matrix = T_final[:3, :3]
    
    # محاور الإحداثيات
    axis_length = 2
    colors = ['red', 'green', 'blue']
    labels = ['X', 'Y', 'Z']
    
    for i in range(3):
        axis_end = end_pos + axis_length * rotation_matrix[:, i]
        ax.plot([end_pos[0], axis_end[0]], 
                [end_pos[1], axis_end[1]], 
                [end_pos[2], axis_end[2]], 
                color=colors[i], linewidth=2, label=f'{labels[i]}-axis')
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    
    # تعديل نسبة العرض
    max_range = np.array([positions[:, 0].max()-positions[:, 0].min(),
                         positions[:, 1].max()-positions[:, 1].min(),
                         positions[:, 2].max()-positions[:, 2].min()]).max() / 2.0
    mid_x = (positions[:, 0].max()+positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max()+positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max()+positions[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    plt.show()

def dh_transform(a, alpha, d, theta):
    """مصفوفة التحويل DH - نسخة مطابقة للملف الأساسي"""
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)
    
    return np.array([
        [cos_theta, -sin_theta*cos_alpha,  sin_theta*sin_alpha, a*cos_theta],
        [sin_theta,  cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
        [0,          sin_alpha,            cos_alpha,            d],
        [0,          0,                    0,                    1]
    ])

def forward_kinematics(joint_angles):
    """Forward Kinematics - نسخة مطابقة للملف الأساسي"""
    DH_params = [
        [0,      np.pi/2,  10,  0],
        [5,      0,        0,   0],
        [10,     0,        0,   0],
        [2,      np.pi/2,  0,   0],
        [1,      0,        0,   0]
    ]
    
    T = np.eye(4)
    for i in range(len(DH_params)):
        a, alpha, d, theta_offset = DH_params[i]
        theta_total = theta_offset + joint_angles[i]
        T = np.dot(T, dh_transform(a, alpha, d, theta_total))
    return T

if __name__ == "__main__":
    # معاملات DH للذراع
    DH_params = [
        [0,      np.pi/2,  10,  0],
        [5,      0,        0,   0],
        [10,     0,        0,   0],
        [2,      np.pi/2,  0,   0],
        [1,      0,        0,   0]
    ]
    
    # مثال على التصور
    joint_angles = [0, np.pi/4, np.pi/6, 0, np.pi/3]
    
    print("رسم الذراع الروبوتية...")
    plot_robot_arm(DH_params, joint_angles, "5-DOF Robot Arm")
