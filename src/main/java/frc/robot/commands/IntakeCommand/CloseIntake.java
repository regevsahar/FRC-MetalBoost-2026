package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;

public class CloseIntake extends Command {
  private final Intake intake;

  public CloseIntake(Intake intake) {
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
    intake.StopMotor();
  }

  @Override
  public boolean isFinished() {
    return intake.isAtTargetPosition(IntakeConstants.TOP_LIMIT);
  }
}
