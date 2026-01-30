package frc.robot.subsystems.Shooter.Hood;

public interface HoodIO {
  public static class HoodIOInputs {
    public double arc = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setTargetArc(double arc) {}

  default void stop() {}
}
