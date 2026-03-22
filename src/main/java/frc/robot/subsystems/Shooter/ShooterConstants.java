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
    // Distance->Angle
    DISTANCE_TO_ARC_MAP.put(1.5, 30.0);
    DISTANCE_TO_ARC_MAP.put(1.5, 38.0);
    DISTANCE_TO_ARC_MAP.put(2.0, 50.0);
    DISTANCE_TO_ARC_MAP.put(2.5, 55.0);
    DISTANCE_TO_ARC_MAP.put(3.0, 64.0);
    DISTANCE_TO_ARC_MAP.put(3.5, 65.0);
    DISTANCE_TO_ARC_MAP.put(4.0, 70.0);
    DISTANCE_TO_ARC_MAP.put(4.5, 75.0);
    DISTANCE_TO_ARC_MAP.put(5.0, 75.0);

    // Distance->RPM

    DISTANCE_TO_RPM_MAP.put(1.5, 1380.0);
    DISTANCE_TO_RPM_MAP.put(1.5, 1400.0);
    DISTANCE_TO_RPM_MAP.put(2.0, 1450.0);
    DISTANCE_TO_RPM_MAP.put(2.5, 1450.0);
    DISTANCE_TO_RPM_MAP.put(3.0, 1545.0);
    DISTANCE_TO_RPM_MAP.put(3.5, 1605.0);
    DISTANCE_TO_RPM_MAP.put(4.0, 1650.0);
    DISTANCE_TO_RPM_MAP.put(4.0, 1750.0);
    DISTANCE_TO_RPM_MAP.put(4.0, 1850.0);
  }

  public static final double kMaxRPM = 5400;
  public static final double kMinRPM = 0;
  public static final double kDefaultRPM = 2500;
  public static final double kRPMPolerance = 200;
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
  public static final int MASTER_MOTOR_PORT = 4;
  public static final int SLAVE_MOTOR_PORT = 15;
  public static final TunableNumber kShooterP = new TunableNumber("Shooter/Real/kP", 0.6947);
  public static final TunableNumber kShooterI = new TunableNumber("Shooter/Real/kI", 0.4);
  public static final TunableNumber kShooterD = new TunableNumber("Shooter/Real/kD", 0.000017);
  public static final TunableNumber kShooterS = new TunableNumber("Shooter/Real/kS", 0.24954);
  public static final TunableNumber kShooterV = new TunableNumber("Shooter/Real/kV", 0.12244);
  public static final TunableNumber kShooterA = new TunableNumber("Shooter/Real/kA", 0.003786);
  public static final double kMaxArc = 80;
  public static final double kMinArc = 30;
  public static final double kArcTolerance = 2;
  public static final int HOOD_MOTOR_ID = 1;
  public static final int HOOD_CANCODER_ID = 51;
  public static final double kHoodGearRatio = 1.25;
  public static final double kHoodRotationsPerDegree =
      (1.0 / 360.0) * kHoodGearRatio; // Rotations per degree of hood
  public static final double kHoodTolerance = 2.0; // Degrees
  public static final boolean kHoodCANcoderInvert = false;
  public static final double kHoodCANcoderOffset = 0.0; // Rotations
  public static final TunableNumber kHoodP = new TunableNumber("Shooter/Hood/kP", 20.0);
  public static final TunableNumber kHoodI = new TunableNumber("Shooter/Hood/kI", 0.5);
  public static final TunableNumber kHoodD = new TunableNumber("Shooter/Hood/kD", 0.5);
  public static final TunableNumber kHoodS = new TunableNumber("Shooter/Hood/kS", 0.0);
  public static final TunableNumber kHoodV = new TunableNumber("Shooter/Hood/kV", 0.0);
  public static final TunableNumber kHoodA = new TunableNumber("Shooter/Hood/kA", 0.0);
  public static final double kHoodStallThreshold = 40.0; // Amps
  public static final TunableNumber kShootRPM = new TunableNumber("Shooter/Real/kShootRPM", 2800);
  public static final TunableNumber kShootArc = new TunableNumber("Shooter/Real/kShootArc", 0);
}
