package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class MBSubsystem extends SubsystemBase {

  private final String logPath;

  public MBSubsystem(String logPath) {
    this.logPath = logPath;
  }

  public String getLogPath() {
    return logPath;
  }

  @Override
  public final void periodic() {
    Command current = super.getCurrentCommand();
    String name = (current != null) ? current.getName() : "None";

    Logger.recordOutput(getLogPath() + "/CurrentCommand", name);

    subsystemPeriodic();
  }

  protected void subsystemPeriodic() {}

  public Command asSubsystemCommand(Command command, String commandName) {
    command.setName(commandName);
    command.addRequirements(this);
    return command;
  }
}
