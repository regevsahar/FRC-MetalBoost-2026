package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.ToleranceMath;
import frc.lib.util.FlightTimeTable;
import frc.lib.util.Leds.LedController;
import frc.lib.util.ShootOnMove.ShotPrediction;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.Swerve.SwerveSub;
import frc.robot.subsystems.Vision.AlignToPoseSub;
import frc.robot.subsystems.Vision.PoseEstimator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootWhileMovingCmd extends Command {
  private final SwerveSub swerve;
  private final PoseEstimator poseEstimator;
  private final AlignToPoseSub alignSubsystem;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final FlyWheelSub flywheel;
  private final HoodSUB hood;
  private final Translation2d target;
  // Optional: visualize future point on the field
  private final Field2d field = new Field2d();

  public ShootWhileMovingCmd(
      SwerveSub swerve,
      PoseEstimator poseEstimator,
      AlignToPoseSub alignSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      FlyWheelSub flywheel,
      HoodSUB hood,
      Translation2d target) {

    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    this.alignSubsystem = alignSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.hood = hood;
    this.flywheel = flywheel;
    this.target = target;

    addRequirements(swerve, alignSubsystem);

    SmartDashboard.putData("ShootOnMoveField", field);
  }

  @Override
  public void initialize() {
    alignSubsystem.resetToCurrent(swerve.getPose());
    LedController.getInstance().startFlashing(edu.wpi.first.wpilibj.util.Color.kGreen);
  }

  @Override
  public void execute() {
    // 1) Current pose
    Pose2d currentPose = poseEstimator.getEstimatedPosition();

    // 2) Distance now -> flight time
    double distanceToHubNow = currentPose.getTranslation().getDistance(this.target);
    double flightTime = FlightTimeTable.get(distanceToHubNow);

    // 3) Robot-relative speeds -> field-relative
    ChassisSpeeds robotRelativeSpeeds = swerve.getRobotRelativeSpeeds();

    // Safer conversion (avoids sign mistakes)
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds,
        currentPose.getRotation());

    // Deadband to reduce noise

    fieldRelativeSpeeds.vxMetersPerSecond = ToleranceMath.applyDeadband(fieldRelativeSpeeds.vxMetersPerSecond,
        Constants.stickDeadband);
    fieldRelativeSpeeds.vyMetersPerSecond = ToleranceMath.applyDeadband(fieldRelativeSpeeds.vyMetersPerSecond,
        Constants.stickDeadband);

    // 4) Predict future position
    Translation2d futurePos = ShotPrediction.predictFuturePosition(currentPose, fieldRelativeSpeeds, flightTime);

    // 5) Record future distance
    double futureDistanceToHub = futurePos.getDistance(this.target);

    // 6) Compute yaw setpoint (what angle you'd aim from futurePos to hub)
    double yawSetpointRad = Math.atan2(this.target.getY() - futurePos.getY(), this.target.getX() - futurePos.getX());
    double yawSetpointDeg = Units.radiansToDegrees(yawSetpointRad);

    // 7) Use your align subsystem for rotation output if you want closed-loop
    // aiming
    Pose2d futurePoseForCalc = new Pose2d(futurePos, currentPose.getRotation());
    double rotationOutput = alignSubsystem.calculateRotationOutput(futurePoseForCalc, this.target);

    // 8) Driver translation
    double xSpeed = translationXSupplier.getAsDouble();
    double ySpeed = translationYSupplier.getAsDouble();

    swerve.drive(new Translation2d(xSpeed, ySpeed), rotationOutput, true, true);

    hood.setTargetDistance(futureDistanceToHub);
    flywheel.setTargetDistance(futureDistanceToHub);

    Logger.recordOutput("ShootWhileMoving/NowDistToHub_m", distanceToHubNow);
    Logger.recordOutput("ShootWhileMoving/FlightTime_s", flightTime);
    Logger.recordOutput("ShootWhileMoving/FieldVx_mps", fieldRelativeSpeeds.vxMetersPerSecond);
    Logger.recordOutput("ShootWhileMoving/FieldVy_mps", fieldRelativeSpeeds.vyMetersPerSecond);
    Logger.recordOutput("ShootWhileMoving/FutureDistToHub_m", futureDistanceToHub);
    Logger.recordOutput("ShootWhileMoving/YawSetpoint_deg", yawSetpointDeg);
    Logger.recordOutput("ShootWhileMoving/RotationOutput", rotationOutput);
    Logger.recordOutput("ShootWhileMoving/CurrentPose", currentPose);
    Logger.recordOutput("ShootWhileMoving/FuturePose", futurePoseForCalc);
    Logger.recordOutput("ShootWhileMoving/TargetPosition", this.target);

    // Field2d visualization
    // TODO: Check what de phuc it is
    field.setRobotPose(currentPose);
    field.getObject("FuturePos").setPose(futurePoseForCalc);
    field.getObject("Target").setPose(new Pose2d(this.target, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    hood.stop();
    swerve.drive(new Translation2d(0, 0), 0, true, false);
    LedController.getInstance().stopFlashing();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
