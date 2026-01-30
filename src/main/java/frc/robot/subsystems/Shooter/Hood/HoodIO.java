package frc.robot.subsystems.Shooter.Hood;

public interface HoodIO {
    public static class HoodIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {
    }

    default void setTargetPosition(double positionRad) {
    }

    default void setVoltage(double volts) {
    }

    default void stop() {
    }
}
