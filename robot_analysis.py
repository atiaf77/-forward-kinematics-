import numpy as np
from forward_kinematics import forward_kinematics, extract_position_and_orientation

def workspace_analysis(DH_params, num_samples=1000):
    """
    تحليل مساحة العمل للذراع الروبوتية
    
    Parameters:
    DH_params: معاملات DH
    num_samples: عدد العينات العشوائية للتحليل
    
    Returns:
    workspace_points: نقاط مساحة العمل
    statistics: إحصائيات مساحة العمل
    """
    workspace_points = []
    
    for _ in range(num_samples):
        # توليد زوايا عشوائية للمفاصل
        joint_angles = np.random.uniform(-np.pi, np.pi, len(DH_params))
        
        # حساب الموضع النهائي
        T = forward_kinematics(joint_angles)
        position, _ = extract_position_and_orientation(T)
        workspace_points.append(position)
    
    workspace_points = np.array(workspace_points)
    
    # حساب الإحصائيات
    statistics = {
        'x_range': [workspace_points[:, 0].min(), workspace_points[:, 0].max()],
        'y_range': [workspace_points[:, 1].min(), workspace_points[:, 1].max()],
        'z_range': [workspace_points[:, 2].min(), workspace_points[:, 2].max()],
        'max_reach': np.max(np.linalg.norm(workspace_points, axis=1)),
        'min_reach': np.min(np.linalg.norm(workspace_points, axis=1)),
        'center': np.mean(workspace_points, axis=0),
        'std': np.std(workspace_points, axis=0)
    }
    
    return workspace_points, statistics

def jacobian_analysis(joint_angles, delta=0.001):
    """
    تحليل مصفوفة الجاكوبيان للذراع
    
    Parameters:
    joint_angles: زوايا المفاصل الحالية
    delta: الفرق الصغير لحساب التفاضل العددي
    
    Returns:
    jacobian: مصفوفة الجاكوبيان
    condition_number: رقم الحالة
    singularity_measure: مقياس التفرد
    """
    # حساب الموضع الحالي
    T_current = forward_kinematics(joint_angles)
    pos_current, _ = extract_position_and_orientation(T_current)
    
    # حساب الجاكوبيان بالتفاضل العددي
    jacobian = np.zeros((3, len(joint_angles)))
    
    for i in range(len(joint_angles)):
        # تعديل زاوية المفصل i
        joint_angles_plus = joint_angles.copy()
        joint_angles_plus[i] += delta
        
        joint_angles_minus = joint_angles.copy()
        joint_angles_minus[i] -= delta
        
        # حساب المواضع الجديدة
        T_plus = forward_kinematics(joint_angles_plus)
        T_minus = forward_kinematics(joint_angles_minus)
        
        pos_plus, _ = extract_position_and_orientation(T_plus)
        pos_minus, _ = extract_position_and_orientation(T_minus)
        
        # حساب التفاضل
        jacobian[:, i] = (pos_plus - pos_minus) / (2 * delta)
    
    # حساب رقم الحالة
    try:
        condition_number = np.linalg.cond(jacobian)
        # حساب مقياس التفرد (determinant of J*J^T)
        JJT = np.dot(jacobian, jacobian.T)
        singularity_measure = np.sqrt(np.linalg.det(JJT))
    except:
        condition_number = np.inf
        singularity_measure = 0
    
    return jacobian, condition_number, singularity_measure

def comprehensive_robot_analysis():
    """تحليل شامل للذراع الروبوتية"""
    print("=" * 60)
    print("التحليل الشامل للذراع الروبوتية 5-DOF")
    print("=" * 60)
    
    # معاملات DH
    DH_params = [
        [0,      np.pi/2,  10,  0],
        [5,      0,        0,   0],
        [10,     0,        0,   0],
        [2,      np.pi/2,  0,   0],
        [1,      0,        0,   0]
    ]
    
    print("\n1. معاملات DH:")
    print("Joint | a    | alpha     | d  | theta_offset")
    print("-" * 45)
    for i, (a, alpha, d, theta) in enumerate(DH_params):
        print(f"  {i+1}   | {a:4.1f} | {alpha:9.4f} | {d:2.0f} | {theta:9.4f}")
    
    # تحليل مساحة العمل
    print("\n2. تحليل مساحة العمل:")
    workspace_points, stats = workspace_analysis(DH_params)
    print(f"نطاق X: [{stats['x_range'][0]:.2f}, {stats['x_range'][1]:.2f}] mm")
    print(f"نطاق Y: [{stats['y_range'][0]:.2f}, {stats['y_range'][1]:.2f}] mm")
    print(f"نطاق Z: [{stats['z_range'][0]:.2f}, {stats['z_range'][1]:.2f}] mm")
    print(f"أقصى مدى: {stats['max_reach']:.2f} mm")
    print(f"أقل مدى: {stats['min_reach']:.2f} mm")
    print(f"المركز: [{stats['center'][0]:.2f}, {stats['center'][1]:.2f}, {stats['center'][2]:.2f}] mm")
    
    # تحليل بعض الأوضاع المحددة
    print("\n3. تحليل أوضاع محددة:")
    test_configurations = [
        ([0, 0, 0, 0, 0], "الوضع الأساسي"),
        ([np.pi/2, 0, 0, 0, 0], "دوران المفصل الأول 90°"),
        ([0, np.pi/2, 0, 0, 0], "رفع المفصل الثاني 90°"),
        ([np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4], "جميع المفاصل 45°")
    ]
    
    for joint_angles, description in test_configurations:
        print(f"\n{description}:")
        T = forward_kinematics(joint_angles)
        pos, rot = extract_position_and_orientation(T)
        print(f"  الموضع: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] mm")
        
        # تحليل الجاكوبيان
        jacobian, cond_num, sing_measure = jacobian_analysis(joint_angles)
        print(f"  رقم حالة الجاكوبيان: {cond_num:.2f}")
        print(f"  مقياس التفرد: {sing_measure:.6f}")
        
        if cond_num > 100:
            print("  تحذير: الذراع قريبة من وضعية التفرد!")
    
    # تحليل الأطوال الكلية
    print("\n4. تحليل الأبعاد:")
    total_length = sum([params[0] for params in DH_params])  # مجموع قيم a
    total_height = sum([params[2] for params in DH_params])  # مجموع قيم d
    print(f"الطول الكلي (مجموع a): {total_length} mm")
    print(f"الارتفاع الكلي (مجموع d): {total_height} mm")
    
    # تحليل درجات الحرية
    print(f"\n5. درجات الحرية: {len(DH_params)} DOF")
    print("القدرات:")
    if len(DH_params) >= 6:
        print("  - موضع وتوجه كامل في الفضاء ثلاثي الأبعاد")
    elif len(DH_params) == 5:
        print("  - موضع كامل مع قيود على التوجه")
    elif len(DH_params) >= 3:
        print("  - موضع في الفضاء ثلاثي الأبعاد")
    else:
        print("  - حركة محدودة")

if __name__ == "__main__":
    comprehensive_robot_analysis()
