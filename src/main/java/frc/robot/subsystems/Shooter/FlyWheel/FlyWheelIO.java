package frc.robot.subsystems.Shooter.FlyWheel;

public interface FlyWheelIO {

  public static class FlyWheelIOInputs {
    // current distance in meters
    public double distance = 0.0;

    // current rpm
    public double rpm = 0.0;
  }

  default void updateInputs(FlyWheelIOInputs inputs) {}

  default void setTargetRPM(double targetRPM) {}

  default void stop() {}
}
