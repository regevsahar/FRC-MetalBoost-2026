package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionConstants {
  public static final class CameraConstants {
    public static final String limelight3name = "limelight-three";
    public static final String limelight4name = "limelight-four";
  }

  public static final class PoseEstimator {
    public static double OdometryFactor = 2;
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.003, 0.003, 0.003);
    public static final Matrix<N3, N1> visionStdDevs =
        VecBuilder.fill(0.05 / OdometryFactor, 0.05 / OdometryFactor, 0.05 / OdometryFactor);
    public static final double stdDevFactor = 0.0035;
    public static final double stdDevFactorTranslation = 0.5;
    public static final double stdDevFactorRotation = 0.5;
    public static final double minimumStdDev = 0.1;
  }

  public static final class AlignToPoseConstants {
    // TODO: Tune these constants
    public static final double kP = 5;
    public static final double kI = 0.01;
    public static final double kD = 0.005;
    public static final double kToleranceRad = Math.toRadians(5);
    public static int kMaxOmegaRadPerSec = 2;
  }
}
