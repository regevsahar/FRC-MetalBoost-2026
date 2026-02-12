package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FlyWheelHoodIntegrationTable;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;

public class FlywheelHoodIntegrationCommand extends Command {
  private final FlyWheelSub flywheel;
  private final HoodSUB hood;

  public FlywheelHoodIntegrationCommand(FlyWheelSub flywheel, HoodSUB hood) {
    this.flywheel = flywheel;
    this.hood = hood;
    addRequirements(flywheel, hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double currentHoodAngle = hood.getCurrentArc();
    double targetRPM = FlyWheelHoodIntegrationTable.getRPMFromAngle(currentHoodAngle);

    flywheel.setTargetRPM(targetRPM);
    hood.setTargetDistance();
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    hood.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
