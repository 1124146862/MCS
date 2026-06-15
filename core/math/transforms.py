import math


EPSILON = 1e-6
Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


def add(left: Vec3, right: Vec3) -> Vec3:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def subtract(left: Vec3, right: Vec3) -> Vec3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def scale(vector: Vec3, factor: float) -> Vec3:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)


def dot(left: Vec3, right: Vec3) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def cross(left: Vec3, right: Vec3) -> Vec3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def length(vector: Vec3) -> float:
    return math.sqrt(dot(vector, vector))


def distance(left: Vec3, right: Vec3) -> float:
    return length(subtract(left, right))


def normalize(vector: Vec3) -> Vec3:
    vector_length = length(vector)
    if vector_length <= EPSILON:
        return (0.0, 0.0, 0.0)
    return (vector[0] / vector_length, vector[1] / vector_length, vector[2] / vector_length)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def lerp(start: Vec3, end: Vec3, factor: float) -> Vec3:
    return (
        start[0] + (end[0] - start[0]) * factor,
        start[1] + (end[1] - start[1]) * factor,
        start[2] + (end[2] - start[2]) * factor,
    )


def normalize_quaternion(quaternion: Quat) -> Quat:
    w, x, y, z = quaternion
    q_length = math.sqrt(w * w + x * x + y * y + z * z)
    if q_length <= EPSILON:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / q_length, x / q_length, y / q_length, z / q_length)


def quaternion_multiply(left: Quat, right: Quat) -> Quat:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def inverse_quaternion(quaternion: Quat) -> Quat:
    w, x, y, z = normalize_quaternion(quaternion)
    return (w, -x, -y, -z)


def quaternion_angle_axis(angle_radians: float, axis: Vec3) -> Quat:
    normalized_axis = normalize(axis)
    if length(normalized_axis) <= EPSILON:
        return (1.0, 0.0, 0.0, 0.0)
    half_angle = angle_radians * 0.5
    sine = math.sin(half_angle)
    return normalize_quaternion(
        (
            math.cos(half_angle),
            normalized_axis[0] * sine,
            normalized_axis[1] * sine,
            normalized_axis[2] * sine,
        )
    )


def quaternion_slerp(start: Quat, end: Quat, factor: float) -> Quat:
    sw, sx, sy, sz = normalize_quaternion(start)
    ew, ex, ey, ez = normalize_quaternion(end)
    dot_value = sw * ew + sx * ex + sy * ey + sz * ez

    if dot_value < 0.0:
        ew, ex, ey, ez = -ew, -ex, -ey, -ez
        dot_value = -dot_value

    if dot_value > 0.9995:
        result = (
            sw + (ew - sw) * factor,
            sx + (ex - sx) * factor,
            sy + (ey - sy) * factor,
            sz + (ez - sz) * factor,
        )
        return normalize_quaternion(result)

    theta_0 = math.acos(clamp(dot_value, -1.0, 1.0))
    theta = theta_0 * factor
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot_value * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (
        sw * s0 + ew * s1,
        sx * s0 + ex * s1,
        sy * s0 + ey * s1,
        sz * s0 + ez * s1,
    )


def extract_twist_rotation(rotation: Quat, axis: Vec3) -> Quat:
    normalized_axis = normalize(axis)
    if length(normalized_axis) <= EPSILON:
        return (1.0, 0.0, 0.0, 0.0)

    w, x, y, z = normalize_quaternion(rotation)
    projected = scale(normalized_axis, dot((x, y, z), normalized_axis))
    return normalize_quaternion((w, projected[0], projected[1], projected[2]))


def rotate_vector(quaternion: Quat, vector: Vec3) -> Vec3:
    pure_vector = (0.0, vector[0], vector[1], vector[2])
    rotated = quaternion_multiply(
        quaternion_multiply(normalize_quaternion(quaternion), pure_vector),
        inverse_quaternion(quaternion),
    )
    return (rotated[1], rotated[2], rotated[3])


def rotation_between_vectors(source: Vec3, target: Vec3) -> Quat:
    normalized_source = normalize(source)
    normalized_target = normalize(target)
    if length(normalized_source) <= EPSILON or length(normalized_target) <= EPSILON:
        return (1.0, 0.0, 0.0, 0.0)

    dot_value = clamp(dot(normalized_source, normalized_target), -1.0, 1.0)
    if dot_value >= 1.0 - EPSILON:
        return (1.0, 0.0, 0.0, 0.0)

    if dot_value <= -1.0 + EPSILON:
        fallback_axis = cross(normalized_source, (0.0, 1.0, 0.0))
        if length(fallback_axis) <= EPSILON:
            fallback_axis = cross(normalized_source, (1.0, 0.0, 0.0))
        return quaternion_angle_axis(math.pi, fallback_axis)

    axis = normalize(cross(normalized_source, normalized_target))
    angle = math.acos(dot_value)
    return quaternion_angle_axis(angle, axis)


def project_on_plane(vector: Vec3, plane_normal: Vec3) -> Vec3:
    normalized_plane = normalize(plane_normal)
    if length(normalized_plane) <= EPSILON:
        return vector
    return subtract(vector, scale(normalized_plane, dot(vector, normalized_plane)))


def angle_between_on_axis(source: Vec3, target: Vec3, axis: Vec3) -> float:
    normalized_axis = normalize(axis)
    projected_source = normalize(project_on_plane(source, normalized_axis))
    projected_target = normalize(project_on_plane(target, normalized_axis))
    if length(projected_source) <= EPSILON or length(projected_target) <= EPSILON:
        return 0.0

    dot_value = clamp(dot(projected_source, projected_target), -1.0, 1.0)
    angle = math.acos(dot_value)
    sign = dot(normalized_axis, cross(projected_source, projected_target))
    return angle if sign >= 0.0 else -angle
