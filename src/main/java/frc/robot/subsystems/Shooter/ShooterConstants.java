package frc.robot.subsystems.Shooter;

public class ShooterConstants {
  // SIM{
  public static final double kMaxRPM = 5000;
  public static final double kMinRPM = 0;
  public static final double kDefaultRPM = 2500;
  public static final double kRPMPolerance = 100;
  public static final double kAngleTolerance = 2;

  // Physics Simulation Constants
  public static final double kGearRatio = 1.0;
  public static final double kMomentOfInertia = 0.01; // kg * m^2
  public static final double kWheelRadius =
      edu.wpi.first.math.util.Units.inchesToMeters(2); // 4 Inch diameter wheel

  // Control Constants
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0;
  public static final double kV = 12.0 / kMaxRPM;
  public static final double kA = 0.0;
  // SIM}
}
