package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSub;

public class OpenIntakeCmd extends Command {
  private final IntakeSub intake;

  public OpenIntakeCmd(IntakeSub intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setTargetPosition(IntakeConstants.TOP_LIMIT);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return intake.isAtTargetPosition(IntakeConstants.TOP_LIMIT);
  }
}
