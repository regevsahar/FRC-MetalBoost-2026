package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ConveyanceCommands.ConveyanceWheelsCmd;
import frc.robot.commands.ConveyanceCommands.RollersBackwardsCmd;
import frc.robot.commands.ShooterCommands.AlignHoodToHubCmd;
import frc.robot.commands.ShooterCommands.ShooterSpeedToHubCmd;
import frc.robot.commands.Vision.AlignToPoseCmd;
import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.Swerve.SwerveSub;
import frc.robot.subsystems.Vision.AlignToPoseSub;
import java.util.function.DoubleSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAutomationCommand extends ParallelCommandGroup {
  public ShooterAutomationCommand(
      FlyWheelSub shooter, HoodSUB hood, ConveyanceSub conveyanceWheels, ConveyanceRollerSub rollers) {

    addCommands(
        new ShooterSpeedToHubCmd(shooter),
        new AlignHoodToHubCmd(hood),
        new ConveyanceWheelsCmd(conveyanceWheels),
        new RollersBackwardsCmd(rollers));
  }

  public ShooterAutomationCommand(
      FlyWheelSub shooter, HoodSUB hood, ConveyanceSub conveyanceWheels, ConveyanceRollerSub rollers,
      SwerveSub swerve, AlignToPoseSub poseAlignSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
    addCommands(
        new ShooterSpeedToHubCmd(shooter),
        new AlignHoodToHubCmd(hood),
        new ConveyanceWheelsCmd(conveyanceWheels),
        new RollersBackwardsCmd(rollers),
        new AlignToPoseCmd(swerve, poseAlignSubsystem, translationSup, strafeSup));
  }
}
