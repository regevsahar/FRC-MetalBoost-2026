package frc.robot.subsystems.Intake;

import frc.lib.util.TunableNumber;

public class IntakeConstants {
  public static final int INTAKE_MOTOR_PORT = 9;
  public static final int INTAKE_ROLLER_PORT = 10;
  public static final TunableNumber kP = new TunableNumber("Intake/kP", 4.5);
  public static final TunableNumber kI = new TunableNumber("Intake/kI", 0.005);
  public static final TunableNumber kD = new TunableNumber("Intake/kD", 0.02);
  public static final TunableNumber kS = new TunableNumber("Intake/kS", 0.85051);
  public static final TunableNumber kV = new TunableNumber("Intake/kV", 0.19712);
  public static final TunableNumber kA = new TunableNumber("Intake/kA", 0.16532);
  public static final TunableNumber kCruiseVelocity = new TunableNumber("Intake/kCruiseVelocity", 300.0);
  public static final TunableNumber kAcceleration = new TunableNumber("Intake/kAcceleration", 50.0);
  public static final TunableNumber kJerk = new TunableNumber("Intake/kJerk", 0.0);
  public static final double kAngleOffset = 0;
  public static final double kTolerance = 0.05;
  public static final double TOP_LIMIT = 4.5; // rotations
  public static final double BOTTOM_LIMIT = 0; // rotations
  public static final double ROTATIONS_PER_CM = 2048;
  public static final double kGearRatio = 1;
  public static final double EJECT_SPEED = -4;
  public static final double INSERT_SPEED = 7;
  public static final double kStallThreshold = 125.0; // Amps
}
