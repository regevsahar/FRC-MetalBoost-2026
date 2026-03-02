package frc.robot.commands.Conveyance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyance.ConveyanceConstants;
import frc.robot.subsystems.Conveyance.ConveyanceSub;

public class ConveyanceWheelsCommand extends Command {

  private ConveyanceSub wheels;

  public ConveyanceWheelsCommand(ConveyanceSub wheels) {
    this.wheels = wheels;
    addRequirements(wheels);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wheels.setSpeed(ConveyanceConstants.kConveyanceSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    wheels.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
