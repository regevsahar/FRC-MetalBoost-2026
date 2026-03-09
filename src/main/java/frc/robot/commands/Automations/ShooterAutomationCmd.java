package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.commands.ConveyanceCommands.ConveyanceWheelsCmd;
import frc.robot.commands.ConveyanceCommands.RollersBackwardsCmd;
import frc.robot.commands.ShooterCommands.AlignHoodToHubCmd;
import frc.robot.commands.ShooterCommands.ShooterSpeedToHubCmd;

import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;

public class ShooterAutomationCmd extends ParallelRaceGroup {

  public ShooterAutomationCmd(
      FlyWheelSub shooter,
      HoodSUB hood,
      ConveyanceSub conveyanceWheels,
      ConveyanceRollerSub rollers) {

    addCommands(
        new ShooterSpeedToHubCmd(shooter, conveyanceWheels),
        new AlignHoodToHubCmd(hood),
        new ConveyanceWheelsCmd(conveyanceWheels),
        new RollersBackwardsCmd(rollers));
  }
}