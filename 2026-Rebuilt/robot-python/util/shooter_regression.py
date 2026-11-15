"""Distance -> shooter velocity regression (port of ShooterRegression.java)."""

import math

from constants import AprilTagConstants

# -- SHOT DATA: distance (m) -> velocity (rps) --------------------------------
# Add more points for better accuracy. Min 3 for quadratic.
DISTANCES = [0.59, 0.649, 0.75, 0.849]  # TODO
VELOCITIES = [28.0, 27.0, 28.0, 33.0]  # TODO

# Polynomial degree: 1=linear, 2=quadratic (recommended), 3=cubic
POLY_DEGREE = 2


def _poly_eval(coeffs: list[float], x: float) -> float:
    """Evaluate polynomial: coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 + ..."""
    return sum(c * x**i for i, c in enumerate(coeffs))


def _poly_fit(xs: list[float], ys: list[float], degree: int) -> list[float]:
    """Least-squares polynomial fit via Gaussian elimination.

    Returns coefficients [c0, c1, c2, ...] such that
    f(x) = c0 + c1*x + c2*x^2 + ... minimizes squared error.
    """
    n = degree + 1
    a = [[0.0] * n for _ in range(n)]
    b = [0.0] * n

    # Build normal equations: A^T A c = A^T y
    for i in range(n):
        for j in range(n):
            a[i][j] = sum(x ** (i + j) for x in xs)
        b[i] = sum((x**i) * y for x, y in zip(xs, ys))

    # Gaussian elimination with partial pivoting
    for col in range(n):
        max_row = max(range(col, n), key=lambda row: abs(a[row][col]))
        a[col], a[max_row] = a[max_row], a[col]
        b[col], b[max_row] = b[max_row], b[col]

        for row in range(col + 1, n):
            f = a[row][col] / a[col][col]
            b[row] -= f * b[col]
            for j in range(col, n):
                a[row][j] -= f * a[col][j]

    # Back-substitution
    coeffs = [0.0] * n
    for i in range(n - 1, -1, -1):
        coeffs[i] = b[i]
        for j in range(i + 1, n):
            coeffs[i] -= a[i][j] * coeffs[j]
        coeffs[i] /= a[i][i]
    return coeffs


# Coefficients computed once at startup
COEFFS = _poly_fit(DISTANCES, VELOCITIES, POLY_DEGREE)


def get_velocity_rotations_per_sec(distance_meters: float) -> float:
    """Required shooter velocity (rps) for a given distance to the target."""
    return _poly_eval(COEFFS, distance_meters)


def get_shooter_distance(ty_degrees: float) -> float:
    """Horizontal distance (meters) to the target from the Limelight's ty."""
    angle_rad = math.radians(AprilTagConstants.CAMERA_ANGLE_DEG + ty_degrees)
    tan = math.tan(angle_rad)
    if abs(tan) < 1e-6:
        # Target is level with the camera's optical axis: distance is unbounded
        return math.inf
    return abs((AprilTagConstants.TARGET_HEIGHT - AprilTagConstants.FRONT_CAMERA_HEIGHT) / tan)
