package frc.robot.Util.Math;

import frc.robot.Constants.AprilTagConstants;

public class ShooterRegression {
    // ── SHOT DATA — distance (m) → velocity (m/s) ───────
  // Add more points for better accuracy. Min 3 for quadratic.
  private static final double[] DISTANCES  = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5};// TODO
  private static final double[] VELOCITIES = {12.1, 14.3, 16.0, 17.4, 18.7, 20.2, 21.8}; //TODO

  // Polynomial degree: 1=linear, 2=quadratic (recommended), 3=cubic
  private static final int POLY_DEGREE = 2;

  // Coefficients computed once at startup
  private static final double[] COEFFS = polyFit(DISTANCES, VELOCITIES, POLY_DEGREE);

  /**
   * Returns the required shooter surface velocity (m/s) for a given
   * distance to target. Uses pre-fit polynomial regression.
   */
  public static double getVelocityMetersPerSec(double distanceMeters) {
    return polyEval(COEFFS, distanceMeters);
  }

  // ── Polynomial helpers ───────────────────────────────

  /** Evaluate polynomial: coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 + ... */
  private static double polyEval(double[] coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.length; i++) {
      result += coeffs[i] * Math.pow(x, i);
    }
    return result;
  }

  /**
   * Least-squares polynomial fit via Gaussian elimination.
   * Returns coefficients [c0, c1, c2, ...] such that
   *   f(x) = c0 + c1*x + c2*x^2 + ...  minimizes squared error.
   */
  private static double[] polyFit(double[] xs, double[] ys, int degree) {
    int n = degree + 1;
    double[][] A = new double[n][n];
    double[]   b = new double[n];

    // Build normal equations: Aᵀ A c = Aᵀ y
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        for (double x : xs) A[i][j] += Math.pow(x, i + j);
      }
      for (int k = 0; k < xs.length; k++) b[i] += Math.pow(xs[k], i) * ys[k];
    }

    // Gaussian elimination with partial pivoting
    for (int col = 0; col < n; col++) {
      int maxRow = col;
      for (int row = col + 1; row < n; row++)
        if (Math.abs(A[row][col]) > Math.abs(A[maxRow][col])) maxRow = row;
      double[] tmp = A[col]; A[col] = A[maxRow]; A[maxRow] = tmp;
      double   tb  = b[col]; b[col]  = b[maxRow]; b[maxRow]  = tb;
      for (int row = col + 1; row < n; row++) {
        double f = A[row][col] / A[col][col];
        b[row] -= f * b[col];
        for (int j = col; j < n; j++) A[row][j] -= f * A[col][j];
      }
    }

    // Back-substitution
    double[] coeffs = new double[n];
    for (int i = n - 1; i >= 0; i--) {
      coeffs[i] = b[i];
      for (int j = i + 1; j < n; j++) coeffs[i] -= A[i][j] * coeffs[j];
      coeffs[i] /= A[i][i];
    }
    return coeffs;
  }

    /**
     * gets LL distance in meters
     * @return
     */
    public static double getShooterDistance(double tyDegrees) {
        double angleRad = Math.toRadians(AprilTagConstants.CAMERA_ANGLE_DEG +tyDegrees);        
        return (AprilTagConstants.TARGET_HEIGHT - AprilTagConstants.CAMERA_ANGLE_DEG) / Math.tan(angleRad);

        
    }
}
