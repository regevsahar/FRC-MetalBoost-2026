package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSub;

public class CloseIntakeCmd extends Command {
  private final IntakeSub intake;

  public CloseIntakeCmd(IntakeSub intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setTargetPosition(IntakeConstants.BOTTOM_LIMIT);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return intake.isAtTargetPosition(IntakeConstants.BOTTOM_LIMIT);
  }
}
