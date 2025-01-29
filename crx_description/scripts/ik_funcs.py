#!/usr/bin/env python

from math import degrees, radians
import numpy as np
# import ikfast_iiwa7_tool as ikv
import random
import time
import math

# Consts etc.

INF = np.inf
PI = np.pi
_EPS = np.finfo(float).eps * 4.0
# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

CART_TOL = 0.2
ANGL_TOL = radians(5)

def getIK_FN():
    ''' Get the inverse kinematics function (depending on whether IKFast is built and available this may be analytical, req. VSC++ on Windows) '''
    def Fn(position=[0,0,0], orientation=[0,0,0,0], *args, **kwargs):
        return ikv.get_ik(position, matrix_from_quat(orientation).tolist())

    return Fn

def get_valid_ik(pose, validate=True, validate_collisions=True, get_one=False, **kwargs):
    solutions = sample_ik(pose, **kwargs)
    # if validate:
    #     solutions = prune_invalid_poses(solutions, cid, botId, get_one, validate_collisions, **kwargs)

    return solutions

def sample_ik(pose, attempts=100, num_candidates=INF, epsilon=CART_TOL, angle=ANGL_TOL, time_limit=INF, **kwargs):     
    gen_pose = get_modded_pose_generator(pose, epsilon=epsilon, angle=angle, **kwargs)
    solutions = []
    ikfast = getIK_FN()

    start_time = time.time()
    if num_candidates != INF:
        attempts = 99999

    for i in range(int(attempts)):
        if time.time() - start_time > time_limit and time_limit != INF:
            print("Time limit reached for IK at pose {}".format(pose))
            break

        pose_test = next(gen_pose)

        ik_result = ikfast(pose_test[0], pose_test[1])
        if ik_result:
            for conf in ik_result:
                solutions.append(conf)
            
            if num_candidates == INF or len(solutions) > num_candidates:
                break

    print("{} solutions found in {} seconds.".format(len(solutions), (time.time() - start_time)))

    if len(solutions) == 0:
        return []
        
    return solutions

def get_modded_pose_generator(pose_base, epsilon=0.1, angle=np.pi/6):
    lower = [-epsilon]*3 + [-angle]*3
    upper = [epsilon]*3 + [angle]*3

    base_pnt = pose_base[0]
    base_euler = euler_from_quat(pose_base[1])
    
    for [x, y, z, roll, pitch, yaw] in interval_generator(lower, upper): # halton?
        pose = Pose(point=[base_pnt[0]+x,base_pnt[1]+y,base_pnt[2]+z], euler=Euler(roll=base_euler[0] + roll, pitch=base_euler[0] + pitch, yaw=base_euler[0] + yaw))
        yield pose

# Utilities and structs (from https://github.com/caelan/pybullet-planning)

def Point(x=0., y=0., z=0.):
    """Representing a point in 3D
    Parameters
    ----------
    x : float, optional
        [description], by default 0.
    y : float, optional
        [description], by default 0.
    z : float, optional
        [description], by default 0.
    Returns
    -------
    np array of three floats
        [description]
    """
    return np.array([x, y, z])

def Euler(roll=0., pitch=0., yaw=0.):
    """Representing a 3D rotation by Eulerian angles
    .. image:: ../images/roll_pitch_yaw.png
        :scale: 60 %
        :align: center
    `image source <https://devforum.roblox.com/t/take-out-pitch-from-rotation-matrix-while-preserving-yaw-and-roll/95204>`_
    Parameters
    ----------
    roll : float, optional
        [description], by default 0.
    pitch : float, optional
        [description], by default 0.
    yaw : float, optional
        [description], by default 0.
    Returns
    -------
    np array of three floats
        [description]
    """
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    """Representing a pose (or frame) in 3D
    Parameters
    ----------
    point : np array of three-floats, optional
        [description], by default None
    euler : np array of three eulerian angles, optional
        (roll, pitch, yaw), by default None
    Returns
    -------
    tuple of point, quaternion
        [description]
    """
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return (point, quat_from_euler(euler[0], euler[1], euler[2]))

def vector_norm(data, axis=None, out=None):
    """Return length, i.e. eucledian norm, of ndarray along axis.
    >>> v = np.random.random(3)
    >>> n = vector_norm(v)
    >>> np.allclose(n, np.linalg.norm(v))
    True
    >>> v = np.random.rand(6, 5, 3)
    >>> n = vector_norm(v, axis=-1)
    >>> np.allclose(n, np.sqrt(np.sum(v*v, axis=2)))
    True
    >>> n = vector_norm(v, axis=1)
    >>> np.allclose(n, np.sqrt(np.sum(v*v, axis=1)))
    True
    >>> v = np.random.rand(5, 4, 3)
    >>> n = np.empty((5, 3), dtype=np.float64)
    >>> vector_norm(v, axis=1, out=n)
    >>> np.allclose(n, np.sqrt(np.sum(v*v, axis=1)))
    True
    >>> vector_norm([])
    0.0
    >>> vector_norm([1.0])
    1.0
    """
    data = np.array(data, dtype=np.float64, copy=True)
    if out is None:
        if data.ndim == 1:
            return math.sqrt(np.dot(data, data))
        data *= data
        out = np.atleast_1d(np.sum(data, axis=axis))
        np.sqrt(out, out)
        return out
    else:
        data *= data
        np.sum(data, axis=axis, out=out)
        np.sqrt(out, out)

def unit_vector(data, axis=None, out=None):
    """Return ndarray normalized by length, i.e. eucledian norm, along axis.
    >>> v0 = np.random.random(3)
    >>> v1 = unit_vector(v0)
    >>> np.allclose(v1, v0 / np.linalg.norm(v0))
    True
    >>> v0 = np.random.rand(5, 4, 3)
    >>> v1 = unit_vector(v0, axis=-1)
    >>> v2 = v0 / np.expand_dims(np.sqrt(np.sum(v0*v0, axis=2)), 2)
    >>> np.allclose(v1, v2)
    True
    >>> v1 = unit_vector(v0, axis=1)
    >>> v2 = v0 / np.expand_dims(np.sqrt(np.sum(v0*v0, axis=1)), 1)
    >>> np.allclose(v1, v2)
    True
    >>> v1 = np.empty((5, 4, 3), dtype=np.float64)
    >>> unit_vector(v0, axis=1, out=v1)
    >>> np.allclose(v1, v2)
    True
    >>> list(unit_vector([]))
    []
    >>> list(unit_vector([1.0]))
    [1.0]
    """
    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data*data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data

def convex_combination(x, y, w=0.5):
    return (1-w)*np.array(x) + w*np.array(y)

def uniform_generator(d):
    while True:
        yield np.random.uniform(size=d)

def halton_generator(d):
    import ghalton
    seed = random.randint(0, 1000)
    #sequencer = ghalton.Halton(d)
    sequencer = ghalton.GeneralizedHalton(d, seed)
    #sequencer.reset()
    while True:
        [weights] = sequencer.get(1)
        yield np.array(weights)

def unit_generator(d, use_halton=True):
    if use_halton:
        try:
            import ghalton
        except ImportError:
            print('ghalton is not installed (https://pypi.org/project/ghalton/)')
            use_halton = False
    return halton_generator(d) if use_halton else uniform_generator(d)

def interval_generator(lower, upper, **kwargs):
    assert len(lower) == len(upper)
    assert np.less_equal(lower, upper).all()
    if np.equal(lower, upper).all():
        return iter([lower])
    return (convex_combination(lower, upper, w=weights) for weights in unit_generator(d=len(lower), **kwargs))

def matrix_from_quat(quaternion):
    """Return homogeneous rotation matrix from quaternion.
    >>> R = matrix_from_quat([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True
    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)[0:3, 0:3]

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.
    axes : One of 24 axis sequences as string or encoded tuple
    Note that many Euler angle triplets can describe one matrix.
    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> np.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not np.allclose(R0, R1): print axes, "failed"
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def euler_from_quat(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.
    >>> angles = euler_from_quat([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(angles, [0.123, 0, 0])
    True
    """
    return euler_from_matrix(matrix_from_quat(quaternion), axes)

def quat_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.
    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple
    >>> q = quat_from_euler(1, 2, 3, 'ryxz')
    >>> np.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
    True
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quaternion = np.empty((4, ), dtype=np.float64)
    if repetition:
        quaternion[i] = cj*(cs + sc)
        quaternion[j] = sj*(cc + ss)
        quaternion[k] = sj*(cs - sc)
        quaternion[3] = cj*(cc - ss)
    else:
        quaternion[i] = cj*sc - sj*cs
        quaternion[j] = cj*ss + sj*cc
        quaternion[k] = cj*cs - sj*sc
        quaternion[3] = cj*cc + sj*ss
    if parity:
        quaternion[j] *= -1

    return quaternion

'''def euler_matrix(ai, aj, ak, axes='sxyz'):
    """Return homogeneous rotation matrix from Euler angles and axis sequence.
    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple
    >>> R = euler_matrix(1, 2, 3, 'syxz')
    >>> np.allclose(np.sum(R[0]), -1.34786452)
    True
    >>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
    >>> np.allclose(np.sum(R[0]), -0.383436184)
    True
    >>> ai, aj, ak = (4.0*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)
    >>> for axes in _TUPLE2AXES.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = np.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M'''

'''def quaternion_about_axis(angle, axis):
    """Return quaternion for rotation about axis.
    >>> q = quaternion_about_axis(0.123, (1, 0, 0))
    >>> np.allclose(q, [0.06146124, 0, 0, 0.99810947])
    True
    """
    quaternion = np.zeros((4, ), dtype=np.float64)
    quaternion[:3] = axis[:3]
    qlen = vector_norm(quaternion)
    if qlen > _EPS:
        quaternion *= math.sin(angle/2.0) / qlen
    quaternion[3] = math.cos(angle/2.0)
    return quaternion'''

'''def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> np.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True
    """
    q = np.empty((4, ), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q'''


'''def quaternion_multiply(quaternion1, quaternion0):
    """Return multiplication of two quaternions.
    >>> q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    >>> np.allclose(q, [-44, -14, 48, 28])
    True
    """
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array((
         x1*w0 + y1*z0 - z1*y0 + w1*x0,
        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
         x1*y0 - y1*x0 + z1*w0 + w1*z0,
        -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=np.float64)'''


'''def quaternion_conjugate(quaternion):
    """Return conjugate of quaternion.
    >>> q0 = random_quaternion()
    >>> q1 = quaternion_conjugate(q0)
    >>> q1[3] == q0[3] and all(q1[:3] == -q0[:3])
    True
    """
    return np.array((-quaternion[0], -quaternion[1],
                        -quaternion[2], quaternion[3]), dtype=np.float64)'''


'''def quaternion_inverse(quaternion):
    """Return inverse of quaternion.
    >>> q0 = random_quaternion()
    >>> q1 = quaternion_inverse(q0)
    >>> np.allclose(quaternion_multiply(q0, q1), [0, 0, 0, 1])
    True
    """
    return quaternion_conjugate(quaternion) / np.dot(quaternion, quaternion)'''


'''def quaternion_slerp(quat0, quat1, fraction, spin=0, shortestpath=True):
    """Return spherical linear interpolation between two quaternions.
    >>> q0 = random_quaternion()
    >>> q1 = random_quaternion()
    >>> q = quaternion_slerp(q0, q1, 0.0)
    >>> np.allclose(q, q0)
    True
    >>> q = quaternion_slerp(q0, q1, 1.0, 1)
    >>> np.allclose(q, q1)
    True
    >>> q = quaternion_slerp(q0, q1, 0.5)
    >>> angle = math.acos(np.dot(q0, q))
    >>> np.allclose(2.0, math.acos(np.dot(q0, q1)) / angle) or \
        np.allclose(2.0, math.acos(-np.dot(q0, q1)) / angle)
    True
    """
    q0 = unit_vector(quat0[:4])
    q1 = unit_vector(quat1[:4])
    if fraction == 0.0:
        return q0
    elif fraction == 1.0:
        return q1
    d = np.dot(q0, q1)
    if abs(abs(d) - 1.0) < _EPS:
        return q0
    if shortestpath and d < 0.0:
        # invert rotation
        d = -d
        q1 *= -1.0
    angle = math.acos(d) + spin * math.pi
    if abs(angle) < _EPS:
        return q0
    isin = 1.0 / math.sin(angle)
    q0 *= math.sin((1.0 - fraction) * angle) * isin
    q1 *= math.sin(fraction * angle) * isin
    q0 += q1
    return q0'''


'''def random_quaternion(rand=None):
    """Return uniform random unit quaternion.
    rand: array like or None
        Three independent random variables that are uniformly distributed
        between 0 and 1.
    >>> q = random_quaternion()
    >>> np.allclose(1.0, vector_norm(q))
    True
    >>> q = random_quaternion(np.random.random(3))
    >>> q.shape
    (4,)
    """
    if rand is None:
        rand = np.random.rand(3)
    else:
        assert len(rand) == 3
    r1 = np.sqrt(1.0 - rand[0])
    r2 = np.sqrt(rand[0])
    pi2 = math.pi * 2.0
    t1 = pi2 * rand[1]
    t2 = pi2 * rand[2]
    return np.array((np.sin(t1)*r1,
                        np.cos(t1)*r1,
                        np.sin(t2)*r2,
                        np.cos(t2)*r2), dtype=np.float64)'''


'''def random_rotation_matrix(rand=None):
    """Return uniform random rotation matrix.
    rnd: array like
        Three independent random variables that are uniformly distributed
        between 0 and 1 for each returned quaternion.
    >>> R = random_rotation_matrix()
    >>> np.allclose(np.dot(R.T, R), np.identity(4))
    True
    """
    return matrix_from_quat(random_quaternion(rand))'''


def main():
    f = getIK_FN()
    
    for x in np.linspace(0.0, 1.0, 50):
        for y in np.linspace(0.0, 1.0, 50):
            for z in np.linspace(0.0, 2.0, 100):
                out = f(position=[1.7, 0.0, 0.0], orientation =[0.0, 0.0, 0.0, 1.0])
                if out != []:
                    print(out)

if __name__ == "__main__":
    main()