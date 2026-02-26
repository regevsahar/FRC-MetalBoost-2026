package frc.robot.subsystems.Intake;

import frc.lib.util.TunableNumber;

public class IntakeConstants {
  public static final int INTAKE_MOTOR_PORT = 14;
  public static final int INTAKE_ROLLER_PORT = 15;
  public static final TunableNumber kP = new TunableNumber("Intake/kP", 0.0);
  public static final TunableNumber kI = new TunableNumber("Intake/kI", 0.0);
  public static final TunableNumber kD = new TunableNumber("Intake/kD", 0.0);
  public static final TunableNumber kS = new TunableNumber("Intake/kS", 0.0);
  public static final TunableNumber kV = new TunableNumber("Intake/kV", 0.0);
  public static final TunableNumber kA = new TunableNumber("Intake/kA", 0.0);
  public static final double kAngleOffset = 0;
  public static final double kTolerance = 0.5; // cm tolerance
  public static final double TOP_LIMIT = 0; // cm
  public static final double BOTTOM_LIMIT = 32; // cm
  public static final double ROTATIONS_PER_CM = 2048;
  public static final double kGearRatio = 1;
  public static final double EJECT_SPEED = 0;
  public static final double INSERT_SPEED = 0;
}