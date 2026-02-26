<<<<<<<< HEAD:src/main/java/frc/robot/commands/Shooter/HoodCommand.java
package frc.robot.commands.Shooter;
========
package frc.robot.commands.ShooterCommands;
>>>>>>>> 560818ccccfb5ff2229af7a41f279b49f7c3f32b:src/main/java/frc/robot/commands/ShooterCommands/HoodCommand.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Hood.HoodSub;

public class HoodCommand extends Command {
  private final HoodSub hood;

  public HoodCommand(HoodSub hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
  }

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
