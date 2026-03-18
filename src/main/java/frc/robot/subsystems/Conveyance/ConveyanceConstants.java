package frc.robot.subsystems.Conveyance;

import edu.wpi.first.wpilibj.I2C;

public class ConveyanceConstants {

  public static final int LOWER_MOTOR_PORT = 2;
  public static final int UPPER_MOTOR_PORT = 14;
  public static final int ROLLERS_PORT_ROLLERS = 11;
  public static final double kConveyanceSpeed = 6;
  public static final double kRollersBackwardsSpeed = -6;
  public static final double kRollersForwardSpeed = 6;
  public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kMXP;
  public static final int BALL_DETECTION_THRESHOLD = 250;
}
