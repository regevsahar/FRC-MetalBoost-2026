package frc.robot.commands.Automations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.ConveyanceCommands.ConveyanceWheelsCmd;
import frc.robot.commands.ConveyanceCommands.RollersBackwardsCmd;
import frc.robot.commands.ShooterCommands.AlignHoodToHubCmd;
import frc.robot.commands.ShooterCommands.ShooterSpeedToHubCmd;
import frc.robot.commands.Vision.AlignToPoseCmd;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;
import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.Swerve.SwerveSub;
import frc.robot.subsystems.Vision.AlignToPoseSub;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterAutomationCmd extends ParallelRaceGroup {

  public ShooterAutomationCmd(
      FlyWheelSub flywheel,
      HoodSUB hood,
      ConveyanceSub conveyanceWheels,
      ConveyanceRollerSub rollers) {

    addCommands(
        new ShooterSpeedToHubCmd(flywheel, conveyanceWheels),
        new AlignHoodToHubCmd(hood),
        new WaitUntilCommand(() -> flywheel.isAtTarget())
            .andThen(new ConveyanceWheelsCmd(conveyanceWheels), new RollersBackwardsCmd(rollers)));
  }

  public ShooterAutomationCmd(
      SwerveSub swerve,
      AlignToPoseSub alignSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      FlyWheelSub flywheel,
      HoodSUB hood,
      ConveyanceSub conveyanceWheels,
      ConveyanceRollerSub rollers,
      Translation2d target) {

    addCommands(
        new ShooterSpeedToHubCmd(flywheel, conveyanceWheels),
        new AlignHoodToHubCmd(hood),
        new AlignToPoseCmd(swerve, alignSubsystem, translationXSupplier, translationYSupplier, target),
        new WaitUntilCommand(() -> flywheel.isAtTarget() && hood.isAtTarget() && alignSubsystem.atSetpoint())
            .andThen(new ConveyanceWheelsCmd(conveyanceWheels), new RollersBackwardsCmd(rollers)));
  }
}
