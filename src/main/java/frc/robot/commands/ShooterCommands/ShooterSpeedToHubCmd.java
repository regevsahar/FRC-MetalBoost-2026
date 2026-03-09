// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;

public class ShooterSpeedToHubCmd extends Command {
  /** Creates a new Shoot. */
  private final FlyWheelSub shooter;
  private final ConveyanceSub conveyance;
  private final Timer noBallTimer = new Timer();
  private boolean timerStarted = false;

  public ShooterSpeedToHubCmd(FlyWheelSub shooter, ConveyanceSub conveyance) {
    this.shooter = shooter;
    this.conveyance = conveyance;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noBallTimer.reset();
    noBallTimer.stop();
    timerStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTargetDistanceFromHub();

    if (!conveyance.hasBall()) {
      if (!timerStarted) {
        noBallTimer.reset();
        noBallTimer.start();
        timerStarted = true;
      }
    } else {
      // Ball detected again — reset the timer
      noBallTimer.stop();
      noBallTimer.reset();
      timerStarted = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    noBallTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timerStarted && noBallTimer.hasElapsed(2.0);
  }
}
