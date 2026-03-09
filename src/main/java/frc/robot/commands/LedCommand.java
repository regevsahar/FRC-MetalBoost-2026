// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Leds.LedController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LedCommand extends Command {
  public enum Mode {FLASH,GRADIENT}
  Mode mode;
  Color[] colors;
  public LedCommand(Mode mode, Color... colors) {
    this.mode = mode;
    this.colors = colors;
    if (colors.length == 0) throw new IllegalArgumentException("Must contain at least 1 color!");
  }

  @Override
  public void initialize() {
    if(mode == Mode.FLASH){
      LedController.getInstance().startFlashing(colors[0]);
    } else if(mode == Mode.GRADIENT){
      LedController.getInstance().startGradient(colors);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if(mode == Mode.FLASH){
      LedController.getInstance().stopFlashing();
    }else if(mode == Mode.GRADIENT){
      LedController.getInstance().stopGradient();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
