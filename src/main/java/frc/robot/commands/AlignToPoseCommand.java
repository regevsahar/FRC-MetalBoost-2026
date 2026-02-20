package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.AlignToPoseSubsystem;

public class AlignToPoseCommand extends Command {

  private final Swerve swerve;
  private final AlignToPoseSubsystem hubAlignSubsystem;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;

  public AlignToPoseCommand(Swerve swerve, AlignToPoseSubsystem hubAlignSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
    this.swerve = swerve;
    this.hubAlignSubsystem = hubAlignSubsystem;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    addRequirements(swerve, hubAlignSubsystem);
  }

  @Override
  public void initialize() {
    hubAlignSubsystem.resetToCurrent(swerve.getPose());
  }

  @Override
  public void execute() {

    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    
    var alliance = DriverStation.getAlliance();

    Translation2d target = Constants.FieldConstants.HUB_CENTER_BLUE;
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      target = Constants.FieldConstants.HUB_CENTER_RED;
    }

    double rotation = hubAlignSubsystem.calculateRotationOutput(
        swerve.getPose(),
        target);

    swerve.drive(new Translation2d(translationVal,strafeVal), rotation, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0, 0.0), 0.0, true, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
