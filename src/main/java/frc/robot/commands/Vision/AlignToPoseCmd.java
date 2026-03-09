package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSub;
import frc.robot.subsystems.Vision.AlignToPoseSub;
import java.util.function.DoubleSupplier;

public class AlignToPoseCmd extends Command {

  private final SwerveSub swerve;
  private final AlignToPoseSub hubAlignSubsystem;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private Translation2d targetPose;

  public AlignToPoseCmd(
      SwerveSub swerve,
      AlignToPoseSub hubAlignSubsystem,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      Translation2d targetPose) {

    this.swerve = swerve;
    this.hubAlignSubsystem = hubAlignSubsystem;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.targetPose = targetPose;
    addRequirements(swerve, hubAlignSubsystem);
  }

  @Override
  public void initialize() {
    hubAlignSubsystem.resetToCurrent(swerve.getPose());
  }

  @Override
  public void execute() {

    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

    var alliance = DriverStation.getAlliance();

    double rotation = hubAlignSubsystem.calculateRotationOutput(swerve.getPose(), this.targetPose);

    swerve.drive(new Translation2d(translationVal, strafeVal), rotation, true, true);
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
