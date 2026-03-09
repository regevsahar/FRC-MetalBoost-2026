package frc.robot.commands.ConveyanceCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyance.ConveyanceConstants;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;

public class RollersBackwardsCmd extends Command {

  private final ConveyanceRollerSub rollers;

  public RollersBackwardsCmd(ConveyanceRollerSub rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    rollers.setSpeed(ConveyanceConstants.kRollersBackwardsSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    rollers.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
