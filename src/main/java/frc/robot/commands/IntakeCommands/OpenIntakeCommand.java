package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class OpenIntakeCommand extends Command {
  private final IntakeSubsystem intake;

  public OpenIntakeCommand(IntakeSubsystem intake) {
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
    intake.StopMotor();
  }

  @Override
  public boolean isFinished() {
    return intake.isAtTargetPosition(IntakeConstants.BOTTOM_LIMIT);
  }
}
