package frc.robot.subsystems.Shooter.Hood;

public class HoodIOSim implements HoodIO {

    private double positionRad = 0.0;
    private double velocityRadPerSec = 0.0;
    private double appliedVolts = 0.0;
    private double currentAmps = 0.0;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRad = positionRad;
        inputs.velocityRadPerSec = velocityRadPerSec;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = currentAmps;
    }

    @Override
    public void setTargetPosition(double positionRad) {
        this.positionRad = positionRad;
    }

    @Override
    public void setVoltage(double volts) {
        this.appliedVolts = volts;
    }

    @Override
    public void stop() {
        this.appliedVolts = 0.0;
    }
}
