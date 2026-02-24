package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.lib.util.TunableNumber;

public class ShooterConstants {

  public static final InterpolatingDoubleTreeMap ANGLE_TO_RPM_MAP =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_ARC_MAP =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM_MAP =
      new InterpolatingDoubleTreeMap();

  static {
    // Angle->RPM
    ANGLE_TO_RPM_MAP.put(38.0, 2800.0);
    ANGLE_TO_RPM_MAP.put(60.0, 3200.0);
    ANGLE_TO_RPM_MAP.put(70.0, 3500.0);
    ANGLE_TO_RPM_MAP.put(82.0, 3400.0);

    // Distance->Angle
    DISTANCE_TO_ARC_MAP.put(0.0, 38.0);
    DISTANCE_TO_ARC_MAP.put(0.5, 42.0);
    DISTANCE_TO_ARC_MAP.put(1.0, 48.0);
    DISTANCE_TO_ARC_MAP.put(1.5, 58.0);
    DISTANCE_TO_ARC_MAP.put(2.0, 67.0);
    DISTANCE_TO_ARC_MAP.put(2.5, 73.0);

    // Distance->RPM
    DISTANCE_TO_RPM_MAP.put(0.0, 2600.0);
    DISTANCE_TO_RPM_MAP.put(0.5, 2675.0);
    DISTANCE_TO_RPM_MAP.put(1.0, 2800.0);
    DISTANCE_TO_RPM_MAP.put(1.5, 3000.0);
    DISTANCE_TO_RPM_MAP.put(2.0, 3250.0);
    DISTANCE_TO_RPM_MAP.put(2.5, 3400.0);
  }

  public static final double kMaxRPM = 5400;
  public static final double kMinRPM = 0;
  public static final double kDefaultRPM = 2500;
  public static final double kRPMPolerance = 300;
  public static final int kNumMotors = 2; // Number of Kraken X60 motors
  public static final double kGearRatio = 1.0;
  public static final double kMomentOfInertia = 0.01; // kg * m^2
  public static final double kMaxVoltage = 16.0; // Maximum voltage limit
  public static final double kLoopTime = 0.02; // Standard loop time of 20ms
  public static final TunableNumber kP = new TunableNumber("Shooter/Sim/kP", 0.01);
  public static final TunableNumber kI = new TunableNumber("Shooter/Sim/kI", 0.0);
  public static final TunableNumber kD = new TunableNumber("Shooter/Sim/kD", 0.0);
  public static final TunableNumber kS = new TunableNumber("Shooter/Sim/kS", 0.0);
  public static final TunableNumber kV = new TunableNumber("Shooter/Sim/kV", 12.0 / kMaxRPM);
  public static final TunableNumber kA = new TunableNumber("Shooter/Sim/kA", 0.0);
  public static final int MASTER_MOTOR_PORT = 9;
  public static final int SLAVE_MOTOR_PORT = 13;
  public static final TunableNumber kShooterP = new TunableNumber("Shooter/Real/kP", 0.6947);
  public static final TunableNumber kShooterI = new TunableNumber("Shooter/Real/kI", 0.55);
  public static final TunableNumber kShooterD = new TunableNumber("Shooter/Real/kD", 0.000017);
  public static final TunableNumber kShooterS = new TunableNumber("Shooter/Real/kS", 0.0);
  public static final TunableNumber kShooterV =
      new TunableNumber("Shooter/Real/kV", 12.0 / kMaxRPM);
  public static final TunableNumber kShooterA = new TunableNumber("Shooter/Real/kA", 0.0);
  public static final double kMaxArc = 82;
  public static final double kMinArc = 30;
  public static final double kArcTolerance = 2.5;
  public static final int HOOD_MOTOR_ID = 0;
  public static final int HOOD_CANCODER_ID = 51;
  public static final double kHoodGearRatio = 1;
  public static final double kHoodRotationsPerDegree =
      (1.0 / 360.0) * kHoodGearRatio; // Rotations per degree of hood
  public static final double kHoodTolerance = 2.0; // Degrees
  public static final boolean kHoodCANcoderInvert = false;
  public static final double kHoodCANcoderOffset = 0.0; // Rotations
  public static final TunableNumber kHoodP = new TunableNumber("Shooter/Hood/kP", 8.0);
  public static final TunableNumber kHoodI = new TunableNumber("Shooter/Hood/kI", 0.5);
  public static final TunableNumber kHoodD = new TunableNumber("Shooter/Hood/kD", 0.5);
  public static final TunableNumber kHoodS = new TunableNumber("Shooter/Hood/kS", 0.0);
  public static final TunableNumber kHoodV = new TunableNumber("Shooter/Hood/kV", 0.0);
  public static final TunableNumber kHoodA = new TunableNumber("Shooter/Hood/kA", 0.0);
}
