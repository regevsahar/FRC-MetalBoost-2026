package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldUtils.FieldPoses;
import frc.lib.util.MapFiltering.FieldGridLoader;
import frc.lib.util.MapFiltering.GridMap;
import frc.lib.util.Paths.PathPlannerUtil;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.Automations.EjectBallsAutomationCmd;
import frc.robot.commands.Automations.InsertBallsAutomationCmd;
import frc.robot.commands.Automations.ResetSubsystemsAutomationCmd;
import frc.robot.commands.Automations.ShooterAutomationCmd;
import frc.robot.commands.ConveyanceCommands.ConveyanceWheelsCmd;
import frc.robot.commands.IntakeCommands.CloseIntakeCmd;
import frc.robot.commands.ResetPositionCommand.ResetIntakeCmd;
import frc.robot.commands.ShooterCommands.AlignHoodToConstValue;
import frc.robot.commands.ShooterCommands.ManualHoodCmd;
import frc.robot.commands.ShooterCommands.ShootConstantValueCmd;
import frc.robot.commands.Swerve.TeleopSwerveCmd;
import frc.robot.commands.Vision.ShootWhileMovingCmd;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;
import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Intake.IntakeRollersSub;
import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIOTalonFX;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSimulation;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOTalonFX;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.Swerve.SwerveSub;
import frc.robot.subsystems.Vision.AlignToPoseSub;
import frc.robot.subsystems.Vision.LimelightSub;
import frc.robot.subsystems.Vision.PoseEstimator;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;
import java.util.Set;

public class RobotContainer {
        /* Controllers */
        private final Joystick driver = new Joystick(0);
        private final Joystick operator = new Joystick(1);

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;

        /* Buttons */
        private final int shootAutomation = XboxController.Axis.kLeftTrigger.value;
        private final int shootWhileMoving = XboxController.Axis.kRightTrigger.value;

        AutoChooser autoChooser;

        /* Subsystems */

        public final PoseEstimator poseEstimator = new PoseEstimator();
        public final LimelightSub limelight = new LimelightSub(CameraConstants.limelight3name);
        public final LimelightSub limelight2 = new LimelightSub(CameraConstants.limelight4name);
        public final AlignToPoseSub AlignToPoseSub = new AlignToPoseSub();
        private final FlyWheelIO shooterIO = RobotBase.isSimulation() ? new FlyWheelSimulation()
                        : new FlyWheelIOTalonFX();

        private final FlyWheelSub shooter = new FlyWheelSub(shooterIO, poseEstimator);

        private final HoodIO hoodIO = RobotBase.isSimulation() ? new HoodIOSim() : new HoodIOTalonFX();

        public final HoodSUB hood = new HoodSUB(hoodIO, poseEstimator);
        public final IntakeSub intake = new IntakeSub();
        private final IntakeRollersSub intakeRollers = new IntakeRollersSub();
        private final ConveyanceSub conveyanceWheels = new ConveyanceSub();
        private final ConveyanceRollerSub rollers = new ConveyanceRollerSub();
        public final SwerveSub s_Swerve = new SwerveSub(poseEstimator);

        /* Driver Buttons */
        private final JoystickButton lowerSwerveSpeed = new JoystickButton(driver,
                        XboxController.Button.kLeftBumper.value);
        private final JoystickButton higherSwerveSpeed = new JoystickButton(driver,
                        XboxController.Button.kRightBumper.value);
        private final JoystickButton followPath = new JoystickButton(driver,
                        XboxController.Button.kB.value);
        private final JoystickButton resetPoseEstimator = new JoystickButton(driver,
                        XboxController.Button.kA.value);

        /* SysId Characterization Buttons (run in Test mode only) */
        // private final JoystickButton sysIdQuasFwd = new JoystickButton(driver,
        // XboxController.Button.kX.value);
        // private final JoystickButton sysIdQuasRev = new JoystickButton(driver,
        // XboxController.Button.kStart.value);
        // private final JoystickButton sysIdDynFwd = new JoystickButton(driver,
        // XboxController.Button.kBack.value);
        // private final Trigger sysIdDynRev = new Trigger(() -> driver.getPOV() == 0);
        // // POV Up

        /// * operation Buttons */
        private final JoystickButton ShootArcValue = new JoystickButton(operator, XboxController.Button.kY.value);
        private final JoystickButton ShootConstantValue = new JoystickButton(operator, XboxController.Button.kX.value);
        private final Trigger shootToZoneTrigger = new Trigger(() -> operator.getRawAxis(shootAutomation) > 0.3);
        private final JoystickButton resetIntakePosition = new JoystickButton(operator, XboxController.Button.kA.value);
        private final JoystickButton openIntake = new JoystickButton(operator,
                        XboxController.Button.kRightBumper.value);
        private final JoystickButton closeIntake = new JoystickButton(operator,
                        XboxController.Button.kLeftBumper.value);
        private final JoystickButton ejectBall = new JoystickButton(operator, XboxController.Button.kB.value);
        private final JoystickButton resetPositionAutomation = new JoystickButton(operator,
                        XboxController.Button.kStart.value); // TODO: check button number
        private final Trigger shootWhileMovingTrigger = new Trigger(() -> operator.getRawAxis(shootWhileMoving) > 0.3);

        public final GridMap fieldGrid;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                fieldGrid = FieldGridLoader.load("FieldGrid.json");

                poseEstimator.addLimelight(limelight);
                poseEstimator.addLimelight(limelight2);

                s_Swerve.setDefaultCommand(
                                new TeleopSwerveCmd(
                                                s_Swerve,
                                                () -> driver.getRawAxis(translationAxis),
                                                () -> driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> true));

                hood.setDefaultCommand(
                                new ManualHoodCmd(hood, () -> operator.getRawAxis(XboxController.Axis.kLeftY.value)));

                // Configure the button bindingsPP
                configureButtonBindings();
                registerPathPlannerCommands();
                autoChooser = new AutoChooser(new PathPlannerAuto("TEST 1M"));
        }

        private void configureButtonBindings() {

                /* Driver Buttons */
                lowerSwerveSpeed.whileTrue(
                                new TeleopSwerveCmd(
                                                s_Swerve,
                                                () -> driver.getRawAxis(translationAxis),
                                                () -> driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> true,
                                                () -> 0.4));

                higherSwerveSpeed.whileTrue(
                                new TeleopSwerveCmd(
                                                s_Swerve,
                                                () -> driver.getRawAxis(translationAxis),
                                                () -> driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> true,
                                                () -> 0.85));

                resetPoseEstimator.onTrue(
                                new InstantCommand(
                                                () -> poseEstimator.sEstimator.resetPosition(
                                                                s_Swerve.getGyroYaw(),
                                                                s_Swerve.getModulePositions(),
                                                                new Pose2d(0, 0, new Rotation2d()))));

                // -----------------------------------------------------------------------
                // SysId — hold each button while enabled in TEST mode on the Driver Station
                // Run all 4 tests, then open the .wpilog in the SysId Analyzer tool.
                // -----------------------------------------------------------------------
                // sysIdQuasFwd.whileTrue(s_Swerve.sysIdQuasistaticForward());
                // sysIdQuasRev.whileTrue(s_Swerve.sysIdQuasistaticReverse());
                // sysIdDynFwd.whileTrue(s_Swerve.sysIdDynamicForward());
                // sysIdDynRev.whileTrue(s_Swerve.sysIdDynamicReverse());

                ShootArcValue.whileTrue(new AlignHoodToConstValue(hood));
                ShootConstantValue.whileTrue(new ShootConstantValueCmd(shooter)
                                .alongWith(new ConveyanceWheelsCmd(conveyanceWheels)));
                resetIntakePosition.onTrue(new ResetIntakeCmd(intake));
                openIntake.whileTrue(new InsertBallsAutomationCmd(intake, intakeRollers));
                closeIntake.whileTrue(new CloseIntakeCmd(intake));
                resetPositionAutomation.whileTrue(new ResetSubsystemsAutomationCmd(hood, intake));
                ejectBall.whileTrue(new EjectBallsAutomationCmd(intakeRollers, rollers, intake));
                shootWhileMovingTrigger.whileTrue(
                                new ShootWhileMovingCmd(
                                                s_Swerve,
                                                poseEstimator,
                                                AlignToPoseSub,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                shooter,
                                                hood,
                                                FieldPoses.getClosestBumper(
                                                                poseEstimator.getEstimatedPosition().getTranslation()),
                                                conveyanceWheels,
                                                rollers));
                shootToZoneTrigger.whileTrue(
                                new ShooterAutomationCmd(
                                                s_Swerve,
                                                AlignToPoseSub,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                shooter,
                                                hood,
                                                conveyanceWheels,
                                                rollers,
                                                FieldPoses.getHubPosByAliiance())); // TODO: change to real target
                followPath.toggleOnTrue(
                                new DeferredCommand(
                                                () -> PathPlannerUtil.createPathDuringRuntime(
                                                                poseEstimator.getEstimatedPosition(),
                                                                new Pose2d(14, 2.48, Rotation2d.fromDegrees(-180)),
                                                                new PathConstraints(4, 2, 2, 2),
                                                                false),
                                                Set.of(s_Swerve)));

                // try {
                // followPath.toggleOnTrue(
                // PathFollowingCommandsBuilder.followPath(
                // PathPlannerPath.fromPathFile("line")
                // )
                // );
                // } catch (Exception e) {
                // // TODO: handle exception
                // }
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelectedAuto();
        }

        void registerPathPlannerCommands() {
        }
}
