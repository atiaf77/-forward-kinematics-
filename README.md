## Forward Kinematics

* Joint 1: Base rotation (yaw)
* Joint 2: Shoulder pitch
* Joint 3: Elbow pitch
* Joint 4: Wrist pitch
* Links: $l_1, l_2, l_3$ (distances between joints)


### Variables

* $\theta_1$: Base yaw angle
* $\theta_2$: Shoulder pitch angle
* $\theta_3$: Elbow pitch angle
* $\theta_4$: Wrist pitch angle
* $l_1, l_2, l_3$: Link lengths

Let:

$$
\phi = \theta_2 + \theta_3 + \theta_4
$$

---

### End-Effector Position

First, compute the horizontal reach $R$ in the $xz$ plane:

$$
R = l_1\cos\theta_2 + l_2\cos(\theta_2+\theta_3) + l_3\cos(\theta_2+\theta_3+\theta_4)
$$

Vertical position ($z$):

$$
z = l_1\sin\theta_2 + l_2\sin(\theta_2+\theta_3) + l_3\sin(\theta_2+\theta_3+\theta_4)
$$

Horizontal coordinates ($x, y$) after base rotation:

$$
\boxed{
\begin{aligned}
x &= \cos\theta_1 \cdot R \\
y &= \sin\theta_1 \cdot R \\
z &= l_1\sin\theta_2 + l_2\sin(\theta_2+\theta_3) + l_3\sin(\theta_2+\theta_3+\theta_4)
\end{aligned}
}

### Homogeneous Transformation Matrix $T_0^4$

The orientation is given by:

$$
R = R_z(\theta_1) \cdot R_y(\phi)
$$

where $\phi = \theta_2+\theta_3+\theta_4$.

Expanded rotation matrix:

$$
R =
\begin{bmatrix}
\cos\theta_1\cos\phi & -\sin\theta_1 & \cos\theta_1\sin\phi \\
\sin\theta_1\cos\phi & \cos\theta_1 & \sin\theta_1\sin\phi \\
-\sin\phi & 0 & \cos\phi
\end{bmatrix}
$$

Complete homogeneous transformation:

$$
T_0^4 =
\begin{bmatrix}
\cos\theta_1\cos\phi & -\sin\theta_1 & \cos\theta_1\sin\phi & x \\
\sin\theta_1\cos\phi & \cos\theta_1 & \sin\theta_1\sin\phi & y \\
-\sin\phi & 0 & \cos\phi & z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Where $x, y, z$ are as defined above.

---

### Notes

* If any joint is **prismatic** instead of revolute, replace the angle variable with a linear displacement in the equations.
* This formulation only assumes position + orientation in 3D space — no wrist roll or gripper rotation.
* You can implement these equations in Python, MATLAB, or Arduino for simulation or control.



