package frc.robot.subsystems.Shooter.Hood;

public interface HoodIO {
  public static class HoodIOInputs {

    public double distance = 0.0;

    public double arc = 0.0;
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrent = 0.0;
    public double tempCelcius = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {
  }

  default void setTargetArc(double arc) {
  }

  default void stop() {
  }
}
