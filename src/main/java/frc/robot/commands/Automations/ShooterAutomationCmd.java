package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.commands.ConveyanceCommands.ConveyanceWheelsCmd;
import frc.robot.commands.ConveyanceCommands.RollersBackwardsCmd;
import frc.robot.commands.ShooterCommands.HoodCmd;
import frc.robot.commands.ShooterCommands.ShooterCmd;

import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.Swerve.SwerveSub;
import frc.robot.subsystems.Vision.AlignToPoseSub;
import java.util.function.DoubleSupplier;

public class ShooterAutomationCmd extends ParallelRaceGroup {

  public ShooterAutomationCmd(
      FlyWheelSub shooter,
      HoodSUB hood,
      ConveyanceSub conveyanceWheels,
      RollersSub rollers) {

    addCommands(
        new ShooterCmd(shooter, conveyanceWheels),
        new HoodCmd(hood),
        new ConveyanceWheelsCmd(conveyanceWheels),
        new RollersBackwardsCmd(rollers));
  }
}