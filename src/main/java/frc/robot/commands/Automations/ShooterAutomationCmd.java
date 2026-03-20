package frc.robot.commands.Automations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ConveyanceCommands.ConveyanceWheelsCmd;
import frc.robot.commands.ConveyanceCommands.RollersBackwardsCmd;
import frc.robot.commands.IntakeCommands.CloseIntakeWithBallsCmd;
import frc.robot.commands.ShooterCommands.AlignHoodToHubCmd;
import frc.robot.commands.ShooterCommands.ShooterSpeedToHubCmd;
import frc.robot.commands.Vision.AlignToPoseCmd;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;
import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.Swerve.SwerveSub;
import frc.robot.subsystems.Vision.AlignToPoseSub;
import java.util.function.DoubleSupplier;

public class ShooterAutomationCmd extends ParallelCommandGroup {

  public ShooterAutomationCmd(
      FlyWheelSub flywheel,
      HoodSUB hood,
      ConveyanceSub conveyanceWheels,
      ConveyanceRollerSub rollers) {

    addCommands(
        new ShooterSpeedToHubCmd(flywheel, conveyanceWheels),
        new AlignHoodToHubCmd(hood),
        new WaitUntilCommand(() -> flywheel.isAtTarget() && hood.isAtTarget())
            .andThen(
                new ConveyanceWheelsCmd(conveyanceWheels)
                    .alongWith(new RollersBackwardsCmd(rollers))));
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
      Translation2d target,
      IntakeSub intake) {

    addCommands(
        new ShooterSpeedToHubCmd(flywheel, conveyanceWheels),
        new AlignHoodToHubCmd(hood),
        new AlignToPoseCmd(
            swerve, alignSubsystem, translationXSupplier, translationYSupplier, target),
        new WaitUntilCommand(
                () -> hood.isAtTarget() && alignSubsystem.atSetpoint() && flywheel.isAtTarget())
            .andThen(
                new ConveyanceWheelsCmd(conveyanceWheels)
                    .alongWith(new RollersBackwardsCmd(rollers)))
            .alongWith(new CloseIntakeWithBallsCmd(intake)));
  }
}
