# 🤖 Forward Kinematics للذراع الروبوتية 5-DOF

مشروع شامل لحساب Forward Kinematics للذراع الروبوتية باستخدام معاملات Denavit-Hartenberg (DH).

## 📋 المحتويات

- [الوصف](#الوصف)
- [المتطلبات](#المتطلبات)
- [التثبيت](#التثبيت)
- [الاستخدام](#الاستخدام)
- [الملفات](#الملفات)
- [الأمثلة](#الأمثلة)
- [المساهمة](#المساهمة)

## 📖 الوصف

هذا المشروع يوفر تطبيقاً كاملاً لحساب Forward Kinematics لذراع روبوتية بـ 5 درجات حرية باستخدام:

- **معاملات Denavit-Hartenberg (DH)**: النهج المعياري لوصف الروابط والمفاصل
- **مصفوفات التحويل المتجانسة**: لحساب المواضع والاتجاهات
- **تحليل شامل**: تحليل مساحة العمل ومصفوفة الجاكوبيان
- **تصور ثلاثي الأبعاد**: رسم الذراع في الفضاء

## 🛠 المتطلبات

```
numpy >= 1.20.0
matplotlib >= 3.3.0
```

## 📦 التثبيت

1. استنساخ المستودع:
```bash
git clone https://github.com/YOUR_USERNAME/robotics-arm-kinematics.git
cd robotics-arm-kinematics
```

2. تثبيت المتطلبات:
```bash
pip install numpy matplotlib
```

## 🚀 الاستخدام

### الاستخدام الأساسي

```python
import numpy as np
from forward_kinematics import forward_kinematics, extract_position_and_orientation

# تعريف زوايا المفاصل (بالـ radians)
joint_angles = [0, np.pi/4, np.pi/6, 0, np.pi/3]

# حساب الوضعية النهائية
end_effector_pose = forward_kinematics(joint_angles)

# استخراج الموضع والاتجاه
position, rotation_matrix = extract_position_and_orientation(end_effector_pose)

print(f"موضع End-Effector: {position}")
print(f"مصفوفة الدوران:\n{rotation_matrix}")
```

### تشغيل الاختبارات

```bash
python forward_kinematics.py
```

### التحليل الشامل

```bash
python robot_analysis.py
```

### التصور ثلاثي الأبعاد

```bash
python robot_visualization.py
```

## 📁 الملفات

| الملف | الوصف |
|-------|--------|
| `forward_kinematics.py` | الملف الرئيسي لحساب Forward Kinematics |
| `robot_analysis.py` | تحليل شامل للذراع ومساحة العمل |
| `robot_visualization.py` | تصور الذراع في الفضاء ثلاثي الأبعاد |
| `README.md` | هذا الملف |
| `requirements.txt` | متطلبات المشروع |

## ⚙️ معاملات DH

المشروع يستخدم معاملات DH التالية للذراع 5-DOF:

| Joint | a (mm) | α (rad) | d (mm) | θ₀ (rad) |
|-------|--------|---------|--------|----------|
| 1     | 0      | π/2     | 10     | 0        |
| 2     | 5      | 0       | 0      | 0        |
| 3     | 10     | 0       | 0      | 0        |
| 4     | 2      | π/2     | 0      | 0        |
| 5     | 1      | 0       | 0      | 0        |

## 📊 الميزات

### ✅ ما يتضمنه المشروع:

- **حساب دقيق**: مصفوفات DH والتحويلات المتجانسة
- **التحقق من الصحة**: اختبارات شاملة لمصفوفات الدوران
- **تحليل متقدم**: مساحة العمل ومصفوفة الجاكوبيان
- **توثيق شامل**: شرح مفصل لكل دالة
- **أمثلة متنوعة**: حالات اختبار مختلفة
- **تصور بصري**: رسم ثلاثي الأبعاد للذراع

### 🎯 النتائج المتوقعة:

- **مدى العمل**: من 1.17mm إلى 27.12mm
- **درجات الحرية**: 5 DOF
- **دقة الحساب**: مصفوفات دوران متعامدة (det=1)

## 🔧 تخصيص المعاملات

لتعديل معاملات DH لذراع مختلفة، قم بتحديث متغير `DH_params` في `forward_kinematics.py`:

```python
DH_params = [
    [a1, alpha1, d1, theta_offset1],
    [a2, alpha2, d2, theta_offset2],
    # ... إضافة المزيد حسب عدد المفاصل
]
```

## 📚 المراجع

- **Denavit-Hartenberg Convention**: الطريقة المعيارية لوصف الروبوتات
- **Homogeneous Transformations**: مصفوفات التحويل في الروبوتات
- **Forward Kinematics**: حساب موضع End-Effector من زوايا المفاصل

## 🤝 المساهمة

نرحب بالمساهمات! يرجى:

1. Fork المستودع
2. إنشاء فرع للميزة الجديدة (`git checkout -b feature/AmazingFeature`)
3. Commit التغييرات (`git commit -m 'Add some AmazingFeature'`)
4. Push إلى الفرع (`git push origin feature/AmazingFeature`)
5. فتح Pull Request

## 📄 الترخيص

هذا المشروع مرخص تحت رخصة MIT - انظر ملف [LICENSE](LICENSE) للتفاصيل.

## 📞 التواصل

إذا كان لديك أي أسئلة أو اقتراحات، لا تتردد في فتح Issue أو التواصل مباشرة.

---

⭐ إذا أعجبك هذا المشروع، لا تنس إعطاؤه نجمة على GitHub!
