package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Conveyance.ConveyanceWheelsCommand;
import frc.robot.commands.Conveyance.RollersBackwardsCommand;
import frc.robot.commands.ShooterCommands.HoodCommand;
import frc.robot.commands.ShooterCommands.ShooterCommand;
import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Conveyance.RollersSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAutomationCommand extends ParallelCommandGroup {
  /** Creates a new GoToL1. */
  public ShooterAutomationCommand(
      FlyWheelSub shooter, HoodSUB hood, ConveyanceSub conveyanceWheels, RollersSub rollers) {

    addCommands(
        new ShooterCommand(shooter),
        new HoodCommand(hood),
        new ConveyanceWheelsCommand(conveyanceWheels),
        new RollersBackwardsCommand(rollers));
  }
}
