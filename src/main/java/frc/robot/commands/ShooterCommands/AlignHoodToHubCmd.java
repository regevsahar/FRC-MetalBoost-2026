package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;

public class AlignHoodToHubCmd extends Command {
  private final HoodSUB hood;

  public AlignHoodToHubCmd(HoodSUB hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hood.setTargetDistance();
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  @Override
  public boolean isFinished() {
    return hood.isAtTarget();
  }
}
