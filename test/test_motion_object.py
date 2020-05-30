import pinocchio as pin
import numpy as np

# Move velocity to a different point with relative transform T
T = pin.SE3(np.eye(3), np.array([1.0, 0.0, 0.0]))
v = pin.Motion(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]))    # rotation only around z
v2 = T.act(v)
print(v)
print(v2)

# Concatenate transforms
T2 = pin.SE3(np.eye(3), np.array([1.0, -1.0, 0.0]))
T3 = T2.act(T)
print(T.translation)
print(T2.translation)
print(T3.translation)


# SE3 differences
p1 = pin.SE3(np.eye(3), np.array([1.0, 0.0, 0.0]))
p2 = pin.SE3(np.eye(3), np.array([3.0, 0.0, 0.0]))
p1_p2 = p1.actInv(p2)
print("p1_p2=\n" + str(p1_p2))

# Velocity which in the time unit brings to
v = pin.log6(p1_p2)
print("v=\n" + str(v))

# SE3 differences with rotation
pitch = np.pi
R = np.matrix([[np.cos(pitch), 0.0, np.sin(pitch)],
               [0, 1, 0],
               [-np.sin(pitch), 0.0, np.cos(pitch)]])

p1 = pin.SE3(np.eye(3), np.array([1.0, 0.0, 0.0]))
p2 = pin.SE3(R, np.array([3.0, 0.0, 0.0]))
p1_p2 = p1.actInv(p2)
print("p1_p2=\n" + str(p1_p2))

# Velocity which in the time unit brings to
v = pin.log6(p1_p2)

# The integral of the velocity in time 1
t = pin.exp6(v)
print("v=\n" + str(v))
print("t=\n" + str(t))


# Test 3
R_1 = np.array([[5.52207e-17, 0,  -1],
                [0, 1, 0],
                [1, 0,  5.52207e-17]])
t_1 = np.array([1.69326e-16, 0, 3])

R_2 = np.array([[6.12323e-17, 0, -1],
                [0, 1, 0],
                [1, 0,  6.12323e-17]])
t_2 = np.array([0.315789, 0, 3])
p1 = pin.SE3(R_1, t_1)
p2 = pin.SE3(R_2, t_2)
p12 = p1.actInv(p2)

print("Test 3 = \n" + str(R_1.dot(p12.translation)))

