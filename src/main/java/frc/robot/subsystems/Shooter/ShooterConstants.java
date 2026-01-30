package frc.robot.subsystems.Shooter;

import frc.robot.util.TunableNumber;

public class ShooterConstants {
  // SIM{

  public static final double kMaxRPM = 5400;
  public static final double kMinRPM = 0;
  public static final double kDefaultRPM = 2500;
  public static final double kRPMPolerance = 300;

  // Physics Simulation Constants
  public static final int kNumMotors = 2; // Number of Kraken X60 motors
  public static final double kGearRatio = 1.0;
  public static final double kMomentOfInertia = 0.01; // kg * m^2
  public static final double kMaxVoltage = 16.0; // Maximum voltage limit
  public static final double kLoopTime = 0.02; // Standard loop time of 20ms

  // Control Constants - Tunable
  public static final TunableNumber kP = new TunableNumber("Shooter/Sim/kP", 0.01);
  public static final TunableNumber kI = new TunableNumber("Shooter/Sim/kI", 0.0);
  public static final TunableNumber kD = new TunableNumber("Shooter/Sim/kD", 0.0);

  public static final TunableNumber kS = new TunableNumber("Shooter/Sim/kS", 0.0);
  public static final TunableNumber kV = new TunableNumber("Shooter/Sim/kV", 12.0 / kMaxRPM);
  public static final TunableNumber kA = new TunableNumber("Shooter/Sim/kA", 0.0);

  // SIM}

  // REAL{

  public static final int MASTER_MOTOR_PORT = 9;
  public static final int SLAVE_MOTOR_PORT = 13;

  public static final TunableNumber kShooterP = new TunableNumber("Shooter/Real/kP", 0.6947);
  public static final TunableNumber kShooterI = new TunableNumber("Shooter/Real/kI", 0.55);
  public static final TunableNumber kShooterD = new TunableNumber("Shooter/Real/kD", 0.000017);
  public static final TunableNumber kShooterS = new TunableNumber("Shooter/Real/kS", 0.0);
  public static final TunableNumber kShooterV =
      new TunableNumber("Shooter/Real/kV", 12.0 / kMaxRPM);
  public static final TunableNumber kShooterA = new TunableNumber("Shooter/Real/kA", 0.0);

  // REAL}

}
