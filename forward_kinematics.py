import numpy as np

def dh_transform(a, alpha, d, theta):
    """
    حساب مصفوفة التحويل باستخدام معاملات DH
    
    Parameters:
    a: طول الرابط (link length)
    alpha: زاوية التواء الرابط (link twist)
    d: إزاحة الرابط (link offset)
    theta: زاوية المفصل (joint angle)
    
    Returns:
    مصفوفة التحويل المتجانسة 4x4
    """
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

# مثال: معاملات DH لذراع 5 DOF (عدل القيم حسب تصميمك)
DH_params = [
    [0,      np.pi/2,  10,  0],  # [a, alpha, d, theta]
    [5,      0,        0,   0],
    [10,     0,        0,   0],
    [2,      np.pi/2,  0,   0],
    [1,      0,        0,   0]
]

def forward_kinematics(joint_angles):
    """
    حساب الوضعية النهائية للذراع (Forward Kinematics)
    
    Parameters:
    joint_angles: قائمة بزوايا المفاصل بالـ radians
    
    Returns:
    مصفوفة التحويل المتجانسة 4x4 للـ end-effector
    """
    if len(joint_angles) != len(DH_params):
        raise ValueError(f"عدد زوايا المفاصل ({len(joint_angles)}) يجب أن يساوي عدد المفاصل ({len(DH_params)})")
    
    T = np.eye(4)  # مصفوفة الهوية
    for i in range(len(DH_params)):
        a, alpha, d, theta_offset = DH_params[i]
        # إضافة زاوية المفصل إلى الزاوية الأساسية
        theta_total = theta_offset + joint_angles[i]
        T = np.dot(T, dh_transform(a, alpha, d, theta_total))
    return T

# مثال استخدام
joint_angles = [0, np.pi/4, np.pi/6, 0, np.pi/3]  # بالـ radians
end_effector_pose = forward_kinematics(joint_angles)
print("End Effector Pose:\n", end_effector_pose)

def extract_position_and_orientation(T):
    """
    استخراج الموضع والاتجاه من مصفوفة التحويل
    
    Parameters:
    T: مصفوفة التحويل المتجانسة 4x4
    
    Returns:
    position: الموضع [x, y, z]
    rotation_matrix: مصفوفة الدوران 3x3
    """
    position = T[:3, 3]
    rotation_matrix = T[:3, :3]
    return position, rotation_matrix

def test_forward_kinematics():
    """اختبارات للتأكد من صحة الكود"""
    print("=" * 50)
    print("اختبار Forward Kinematics")
    print("=" * 50)
    
    # اختبار 1: جميع المفاصل في الوضع الصفري
    print("\nاختبار 1: جميع المفاصل في الوضع الصفري")
    joint_angles_zero = [0, 0, 0, 0, 0]
    T_zero = forward_kinematics(joint_angles_zero)
    pos_zero, rot_zero = extract_position_and_orientation(T_zero)
    print(f"الموضع: {pos_zero}")
    print(f"مصفوفة الدوران:\n{rot_zero}")
    
    # اختبار 2: زوايا مختلفة
    print("\nاختبار 2: زوايا مختلفة")
    joint_angles_test = [np.pi/6, np.pi/4, np.pi/3, np.pi/2, np.pi/4]
    T_test = forward_kinematics(joint_angles_test)
    pos_test, rot_test = extract_position_and_orientation(T_test)
    print(f"زوايا المفاصل (degrees): {np.degrees(joint_angles_test)}")
    print(f"الموضع: {pos_test}")
    print(f"مصفوفة الدوران:\n{rot_test}")
    
    # اختبار 3: التحقق من خصائص مصفوفة الدوران
    print("\nاختبار 3: التحقق من خصائص مصفوفة الدوران")
    det_rot = np.linalg.det(rot_test)
    is_orthogonal = np.allclose(np.dot(rot_test, rot_test.T), np.eye(3))
    print(f"محدد مصفوفة الدوران: {det_rot:.6f} (يجب أن يكون 1)")
    print(f"هل المصفوفة متعامدة؟ {is_orthogonal}")

# تشغيل الاختبارات
test_forward_kinematics()
