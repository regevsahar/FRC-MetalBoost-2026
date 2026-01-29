package frc.robot.subsystems.Shooter;

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

      // Control Constants
      public static final double kP = 0.01;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double kS = 0.0;
      public static final double kV = 12.0 / kMaxRPM;
      public static final double kA = 0.0;

  // SIM}

  //REAL{

      public static final int MASTER_MOTOR_PORT =9;
      public static final int SLAVE_MOTOR_PORT =13;

      public static final double kShooterP = 0.6947;
      public static final double kShooterI = 0.55;
      public static final double kShooterD = 0.000017;
      public static final double kShooterS = 0;
      public static final double kShooterV = 12.0 / kMaxRPM;
      public static final double kShooterA = 0;

  //REAL}

}
