package frc.robot.commands.ResetPositionCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSub;

public class ResetIntakeCmd extends Command {
    private final IntakeSub intake;

    public ResetIntakeCmd(IntakeSub intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setMotor(-0.2); // :TODO: "tune" this
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        intake.resetPosition();
    }

    @Override
    public boolean isFinished() {
        return intake.isStalling();
    }
}
