package frc.robot.commands.Conveyance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyance.ConveyanceConstants;
import frc.robot.subsystems.Conveyance.RollersSub;

public class RollersBackwardsCommand extends Command {

  private final RollersSub rollers;

  public RollersBackwardsCommand(RollersSub rollers) {
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
