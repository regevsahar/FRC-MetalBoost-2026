package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSub;

public class CloseIntakeWithBallsCmd extends Command {
    private final IntakeSub intake;

    public CloseIntakeWithBallsCmd(IntakeSub intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (intake.isStalling()) {
            intake.setTargetPosition(intake.getCurrentPosition() + 0.4);
        } else {
            intake.setTargetPosition(IntakeConstants.ROLLERS_LIMIT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.isAtTargetPosition(IntakeConstants.ROLLERS_LIMIT);
    }
}
