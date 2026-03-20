package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleCommand extends Command {
  private final Joystick driver;
  private final Joystick operator;

  public RumbleCommand(Joystick driver, Joystick operator) {
    this.driver = driver;
    this.operator = operator;
    // Do not add requirements, so this can run parallel to anything
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Keep rumbling just in case
    driver.setRumble(RumbleType.kBothRumble, 1.0);
    operator.setRumble(RumbleType.kBothRumble, 1.0);
  }

  @Override
  public void end(boolean interrupted) {
    driver.setRumble(RumbleType.kBothRumble, 0.0);
    operator.setRumble(RumbleType.kBothRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
