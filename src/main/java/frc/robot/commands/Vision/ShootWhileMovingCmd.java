package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FlightTimeTable;
import frc.lib.util.ShotPrediction;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.Swerve.SwerveSub;
import frc.robot.subsystems.Vision.AlignToPoseSub;
import frc.robot.subsystems.Vision.PoseEstimator;
import frc.robot.subsystems.Vision.VisionConstants.FieldConstants;
import java.util.Optional;
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

  // Optional: visualize future point on the field
  private final Field2d field = new Field2d();

  public ShootWhileMovingCmd(
      SwerveSub swerve,
      PoseEstimator poseEstimator,
      AlignToPoseSub alignSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      FlyWheelSub flywheel,
      HoodSUB hood) {

    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    this.alignSubsystem = alignSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.hood = hood;
    this.flywheel = flywheel;

    addRequirements(swerve, alignSubsystem);

    SmartDashboard.putData("ShootOnMoveField", field);
  }

  @Override
  public void initialize() {
    alignSubsystem.resetToCurrent(swerve.getPose());
  }

  @Override
  public void execute() {
    // 1) Current pose
    Pose2d currentPose = poseEstimator.getEstimatedPosition();

    // 2) Hub position by alliance
    Translation2d hubPosition = FieldConstants.HUB_CENTER_BLUE;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      hubPosition = FieldConstants.HUB_CENTER_RED;
    }

    // 3) Distance now -> flight time
    double distanceToHubNow = currentPose.getTranslation().getDistance(hubPosition);
    double flightTime = FlightTimeTable.get(distanceToHubNow);

    // 4) Robot-relative speeds -> field-relative
    ChassisSpeeds robotRelativeSpeeds = swerve.getRobotRelativeSpeeds();

    // Safer conversion (avoids sign mistakes)
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());

    // Deadband to reduce noise
    if (Math.abs(fieldRelativeSpeeds.vxMetersPerSecond) < 0.1)
      fieldRelativeSpeeds.vxMetersPerSecond = 0;
    if (Math.abs(fieldRelativeSpeeds.vyMetersPerSecond) < 0.1)
      fieldRelativeSpeeds.vyMetersPerSecond = 0;

    // 5) Predict future position
    Translation2d futurePos =
        ShotPrediction.predictFuturePosition(currentPose, fieldRelativeSpeeds, flightTime);

    // 6) Record future distance
    double futureDistanceToHub = futurePos.getDistance(hubPosition);

    // 7) Compute yaw setpoint (what angle you'd aim from futurePos to hub)
    double yawSetpointRad =
        Math.atan2(hubPosition.getY() - futurePos.getY(), hubPosition.getX() - futurePos.getX());
    double yawSetpointDeg = Units.radiansToDegrees(yawSetpointRad);

    // 8) Use your align subsystem for rotation output if you want closed-loop
    // aiming
    Pose2d futurePoseForCalc = new Pose2d(futurePos, currentPose.getRotation());
    double rotationOutput = alignSubsystem.calculateRotationOutput(futurePoseForCalc, hubPosition);

    // 9) Driver translation
    double xSpeed = translationXSupplier.getAsDouble();
    double ySpeed = translationYSupplier.getAsDouble();

    swerve.drive(new Translation2d(xSpeed, ySpeed), rotationOutput, true, true);

    hood.setTargetDistance();
    flywheel.setTargetDistance();

    Logger.recordOutput("ShootWhileMoving/NowDistToHub_m", distanceToHubNow);
    Logger.recordOutput("ShootWhileMoving/FlightTime_s", flightTime);
    Logger.recordOutput("ShootWhileMoving/FieldVx_mps", fieldRelativeSpeeds.vxMetersPerSecond);
    Logger.recordOutput("ShootWhileMoving/FieldVy_mps", fieldRelativeSpeeds.vyMetersPerSecond);
    Logger.recordOutput("ShootWhileMoving/FutureDistToHub_m", futureDistanceToHub);
    Logger.recordOutput("ShootWhileMoving/YawSetpoint_deg", yawSetpointDeg);
    Logger.recordOutput("ShootWhileMoving/RotationOutput", rotationOutput);
    Logger.recordOutput("ShootWhileMoving/CurrentPose", currentPose);
    Logger.recordOutput("ShootWhileMoving/FuturePose", futurePoseForCalc);
    Logger.recordOutput("ShootWhileMoving/HubPosition", hubPosition);

    // Field2d visualization
    // TODO: Check what de phuc it is
    field.setRobotPose(currentPose);
    field.getObject("FuturePos").setPose(new Pose2d(futurePos, currentPose.getRotation()));
    field.getObject("Hub").setPose(new Pose2d(hubPosition, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    hood.stop();
    swerve.drive(new Translation2d(0, 0), 0, true, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
