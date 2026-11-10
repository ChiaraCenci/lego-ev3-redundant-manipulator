#!/usr/bin/env python3
"""Original kinematic EV3 controller with import-safe setup and exception cleanup.
Joint logs: radians; Cartesian logs: metres; dt: clipped integration seconds.
The implemented first-joint gear ratio is 12/36. Hardware Verificationtion required.
"""
import time
import numpy as np
l1 = 0.08
l2 = 0.0
l3 = 0.0
l3_ = 0.0
l4 = 0.085
d1 = 0.0
d2 = 0.0
d3 = 0.045
d3_ = 0.11
d4 = 0.0
alpha1 = -np.pi / 2
alpha2 = np.pi / 2
alpha3 = -np.pi / 2
alpha3_ = 0.0
alpha4 = 0.0
GEAR_Q1_FROM_M1 = 12.0 / 36.0
q_min = np.radians(np.array([-120, -20, -124, -180]))
q_max = np.radians(np.array([90, 35, 45, 180]))
A = np.array([0.165, 0.11, 0.185])
B = np.array([0.072, 0.184, 0.178])
C = np.array([-0.08, 0.185, 0.183])
motor_speed = 15
SLEEP_LOOP = 0.01
kgain = 3
t0_traj = 0.0
t1_traj = 12.0
t2_traj = 24.0
DT_MIN = 0.0001
DT_MAX = 0.1
QDOT_MAX = np.radians(15)
DELTA_Q_MAX = np.radians(3)

def DH_computation(d, a, alpha, theta):
    return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)], [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)], [0.0, np.sin(alpha), np.cos(alpha), d], [0.0, 0.0, 0.0, 1.0]])

def forward_kinematics(q1, q2, q3, q4):
    Tb0 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.14], [0, 0, 0, 1]])
    T01 = DH_computation(d1, l1, alpha1, q1)
    T12 = DH_computation(d2, l2, alpha2, q2)
    T23 = DH_computation(d3, l3, alpha3, q3)
    T23_ = DH_computation(d3_, l3_, alpha3_, 0.0)
    T34 = DH_computation(d4, l4, alpha4, q4)
    Tb1 = Tb0 @ T01
    Tb2 = Tb1 @ T12
    Tb3 = Tb2 @ T23
    Tb3_f = Tb3 @ T23_
    Tb4 = Tb3_f @ T34
    p0 = Tb0[:3, 3]
    p1 = Tb1[:3, 3]
    p2 = Tb2[:3, 3]
    p3 = Tb3[:3, 3]
    p3_f = Tb3_f[:3, 3]
    p4 = Tb4[:3, 3]
    return ((Tb0, Tb1, Tb2, Tb3, Tb3_f, Tb4), (p0, p1, p2, p3, p3_f, p4))

def getEndEffector3D(q):
    _, (_, _, _, _, _, p4) = forward_kinematics(q[0], q[1], q[2], q[3])
    return p4.copy()

def Jacobian(q):
    (Tb0, Tb1, Tb2, Tb3, Tb3_f, Tb4), (p0, p1, p2, p3, p3_f, p4) = forward_kinematics(q[0], q[1], q[2], q[3])
    z0 = Tb0[:3, 2]
    z1 = Tb1[:3, 2]
    z2 = Tb2[:3, 2]
    z3 = Tb3_f[:3, 2]
    J = np.zeros((3, 4))
    J[:, 0] = np.cross(z0, p4 - p0)
    J[:, 1] = np.cross(z1, p4 - p1)
    J[:, 2] = np.cross(z2, p4 - p2)
    J[:, 3] = np.cross(z3, p4 - p3_f)
    return J

def saturateJoints(q):
    return np.clip(q, q_min, q_max)

def read_motor_deg():
    return np.array([m1.position, m2.position, m3.position, m4.position], dtype=float)

def read_joint_deg():
    motor_deg = read_motor_deg()
    q1 = GEAR_Q1_FROM_M1 * motor_deg[0]
    q2 = motor_deg[1]
    q3 = motor_deg[2]
    q4 = motor_deg[3]
    return np.array([q1, q2, q3, q4], dtype=float)

def joint_to_motor_deg(joint_deg):
    motor1 = joint_deg[0] / GEAR_Q1_FROM_M1
    motor2 = joint_deg[1]
    motor3 = joint_deg[2]
    motor4 = joint_deg[3]
    return np.array([motor1, motor2, motor3, motor4], dtype=float)

def send_motor_targets(target_deg, speed=motor_speed):
    speed_m1 = 3 * speed
    m1.on_to_position(speed_m1, target_deg[0], brake=True, block=False)
    m2.on_to_position(speed, target_deg[1], brake=True, block=False)
    m3.on_to_position(speed, target_deg[2], brake=True, block=False)
    m4.on_to_position(speed, target_deg[3], brake=True, block=False)

def stop_all():
    for motor in motors:
        try:
            motor.off(brake=True)
        except Exception as exc:
            print('Motor stop failed:', exc)

def cartesianPlanner(pi, pf, ti, tf, t):
    if t <= ti:
        return (pi.copy(), np.zeros(3))
    if t <= tf:
        A_mat = np.array([[1.0, ti, ti ** 2, ti ** 3], [0.0, 1.0, 2.0 * ti, 3.0 * ti ** 2], [1.0, tf, tf ** 2, tf ** 3], [0.0, 1.0, 2.0 * tf, 3.0 * tf ** 2]])
        L = np.linalg.norm(pf - pi)
        if L < 1e-12:
            return (pi.copy(), np.zeros(3))
        b = np.array([0.0, 0.0, L, 0.0])
        coeff = np.linalg.solve(A_mat, b)
        s = coeff[0] + coeff[1] * t + coeff[2] * t ** 2 + coeff[3] * t ** 3
        sdot = coeff[1] + 2.0 * coeff[2] * t + 3.0 * coeff[3] * t ** 2
        direction = (pf - pi) / L
        pd = pi + direction * s
        pdot = direction * sdot
        return (pd, pdot)
    return (pf.copy(), np.zeros(3))

def desired_cartesian_trajectory(t):
    if t <= t1_traj:
        return cartesianPlanner(A, B, t0_traj, t1_traj, t)
    elif t <= t2_traj:
        return cartesianPlanner(B, C, t1_traj, t2_traj, t)
    else:
        return (C.copy(), np.zeros(3))

def inverseKinematics(q_real, xd, xdot_d, k=kgain):
    J = Jacobian(q_real)
    x_real = getEndEffector3D(q_real)
    err = xd - x_real
    K = np.diag([k, k, k * 1.5])
    qdot_cmd = np.linalg.pinv(J) @ (xdot_d + K @ err)
    qdot_cmd = np.clip(qdot_cmd, -QDOT_MAX, QDOT_MAX)
    return (qdot_cmd, err)

def main():
    global m1, m2, m3, m4, motors
    motors = []
    Pd_log, Preal_log, dt_log, qdes_log, qreal_log, qdot_log = ([], [], [], [], [], [])
    try:
        from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
        m1 = LargeMotor(OUTPUT_A)
        motors.append(m1)
        m2 = LargeMotor(OUTPUT_B)
        motors.append(m2)
        m3 = LargeMotor(OUTPUT_C)
        motors.append(m3)
        m4 = MediumMotor(OUTPUT_D)
        motors.append(m4)
        m1.polarity = 'inversed'
        m2.polarity = 'normal'
        m3.polarity = 'normal'
        m4.polarity = 'inversed'
        m1.reset()
        m2.reset()
        m3.reset()
        m4.reset()
        time.sleep(0.5)
        print('reset completed.')
        motor_deg = read_motor_deg()
        joint_deg = read_joint_deg()
        q_real = np.radians(joint_deg)
        q_cmd = q_real.copy()
        qdot = np.zeros(4)
        t_start = time.time()
        t_prev = t_start
        while True:
            t_now = time.time()
            tempo = t_now - t_start
            dt = t_now - t_prev
            t_prev = t_now
            dt = min(max(dt, DT_MIN), DT_MAX)
            if tempo > t2_traj:
                print('online trajectory complete.')
                break
            p_des, pdot_des = desired_cartesian_trajectory(tempo)
            motor_deg = read_motor_deg()
            joint_deg = read_joint_deg()
            q_real = np.radians(joint_deg)
            qdot_cmd, err_p = inverseKinematics(q_real, p_des, pdot_des, k=kgain)
            qdot = qdot_cmd.copy()
            delta_q = dt * qdot
            delta_q = np.clip(delta_q, -DELTA_Q_MAX, DELTA_Q_MAX)
            q_cmd = saturateJoints(q_real + delta_q)
            target_joint_deg = np.degrees(q_cmd)
            target_motor_deg = joint_to_motor_deg(target_joint_deg)
            send_motor_targets(target_motor_deg, speed=motor_speed)
            p_real = getEndEffector3D(q_real)
            err_p_norm = np.linalg.norm(err_p)
            Pd_log.append(p_des.copy())
            Preal_log.append(p_real.copy())
            dt_log.append(dt)
            qdes_log.append(q_cmd.copy())
            qreal_log.append(q_real.copy())
            qdot_log.append(qdot.copy())
            time.sleep(SLEEP_LOOP)
    except KeyboardInterrupt:
        print('Interrupted by keyboard')
    finally:
        stop_all()
        if Pd_log:
            for suffix, values in [('Pd_xyz', Pd_log), ('Preal_xyz', Preal_log), ('dt_reale', dt_log), ('Qdes', qdes_log), ('Qreal', qreal_log), ('Qdot', qdot_log)]:
                np.savetxt('CENCI3v_' + suffix + '_online.txt', np.asarray(values))
if __name__ == '__main__':
    main()
