package frc.robot.subsystems.Shooter.FlyWheel;

public interface FlyWheelIO {

  public static class ShooterIOInputs {

    // current distance in meters
    public double distance = 0.0;

    // current rpm
    public double rpm = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTargetRPM(double targetRPM) {}

  default void stop() {}
}
