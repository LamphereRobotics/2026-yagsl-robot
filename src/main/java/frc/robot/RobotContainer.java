// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.subsystems.extendo.Extendo;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        // #region Controllers
        // Replace with CommandPS4Controller or CommandJoystick if needed
        final CommandXboxController driverXbox = new CommandXboxController(0);
        final CommandXboxController operatorXbox = new CommandXboxController(1);
        // #endregion

        // #region Subsystems
        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve"));
        private final Shooter shooter = new Shooter();
        private final Extendo extendo = new Extendo();
        private final Intake intake = new Intake();
        private final Hopper hopper = new Hopper();
        // #endregion

        // #region Autonomous Setup
        // Establish a Sendable Chooser that will be able to be sent to the
        // SmartDashboard, allowing selection of desired auto
        private final SendableChooser<Command> autoChooser;
        // #endregion

        // #region Driver Commands
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(() -> -driverXbox.getRightX())
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        SwerveInputStream driveAim = driveAngularVelocity.copy()
                        .aimHeadingOffset(Rotation2d.k180deg)
                        .aimHeadingOffset(true)
                        .aimWhile(true);
        SwerveInputStream driveAimBlueHubInputs = driveAim.copy()
                        .aim(TargetConstants.blueHubPose);
        SwerveInputStream driveAimRedHubInputs = driveAim.copy()
                        .aim(TargetConstants.redHubPose);

        Command driveFieldOrientedAngularVelocity() {
                return drivebase.driveFieldOriented(driveAngularVelocity);
        }

        Command driveAimHub() {
                return Commands.either(
                                drivebase.driveFieldOriented(driveAimRedHubInputs),
                                drivebase.driveFieldOriented(driveAimBlueHubInputs),
                                drivebase::isRedAlliance);
        }

        Command driveSlowCommand() {
                return Commands.runEnd(
                                drivebase.setMaxSpeed(Constants.MAX_SPEED * 0.5),
                                drivebase::resetMaxSpeed);
        }

        Command lockDrive() {
                return Commands.runOnce(drivebase::lock, drivebase)
                                .repeatedly();
        }

        Command zeroGyro() {
                return Commands.runOnce(drivebase::zeroGyroWithAlliance);
        }
        // #endregion

        // #region Operator Commands
        Command moveExtendoJoystick() {
                return extendo.moveCommand(() -> operatorXbox.getLeftY());
        }

        Command spinUpBlindCommand() {
                return shooter.shootBlindCommand()
                                .until(shooter::isReadyToShoot);
        }

        Command spinUpFerryCommand() {
                return shooter.shootFerryCommand()
                                .until(shooter::isReadyToShoot);
        }

        Command spinUpHubCommand() {
                return shooter.shootHubCommand(drivebase::getDistanceToHub)
                                .until(shooter::isReadyToShoot);
        }

        Command shootBlindAndFeedCommand() {
                return shooter.shootBlindCommand()
                                .alongWith(hopper.inCommand());
        }

        Command shootHubAndFeedCommand() {
                return shooter.shootHubCommand(drivebase::getDistanceToHub)
                                .alongWith(hopper.inCommand());
        }

        Command shootFerryAndFeedCommand() {
                return shooter.shootFerryCommand()
                                .alongWith(hopper.inCommand());
        }

        Command shootBlindSequenceCommand() {
                return spinUpBlindCommand()
                                .andThen(shootBlindAndFeedCommand());
        }

        Command shootHubSequenceCommand() {
                return spinUpHubCommand()
                                .andThen(shootHubAndFeedCommand());
        }

        Command shootFerrySequenceCommand() {
                return spinUpFerryCommand()
                                .andThen(shootFerryAndFeedCommand());
        }

        Command agitateCommand() {
                return extendo.retractCommand()
                                .until(extendo::isAgitateRetracted)
                                .andThen(extendo.extendCommand()
                                                .until(extendo::isAgitateExtend))
                                .repeatedly();
        }

        Command intakeAndSlowCommand() {
                return lowerAndIntakeCommand()
                                .alongWith(Commands.runEnd(
                                                drivebase.setMaxSpeed(Constants.MAX_SPEED * 0.5),
                                                drivebase::resetMaxSpeed));
        }

        Command shootAndAgitateSequenceCommand() {
                return shootBlindSequenceCommand()
                                .alongWith(agitateCommand());
        }

        Command lowerAndIntakeCommand() {
                return extendo.extendCommand()
                                .alongWith(intake.inCommand());
        }

        void stopAll() {
                shooter.stop();
                extendo.stop();
                intake.stop();
                hopper.stop();
        }
        // #endregion

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);

                // #region Autonomous Commands
                // Create the NamedCommands that will be used in PathPlanner
                NamedCommands.registerCommand("Shoot",
                                shootAndAgitateSequenceCommand()
                                                .withTimeout(6.0)
                                                .finallyDo(this::stopAll));

                NamedCommands.registerCommand("Extend",
                                extendo.extendCommand()
                                                .until(extendo::isFullyExtended)
                                                .finallyDo(this::stopAll));

                NamedCommands.registerCommand("Intake",
                                lowerAndIntakeCommand()
                                                .finallyDo(this::stopAll));

                // Have the autoChooser pull in all PathPlanner autos as options
                autoChooser = AutoBuilder.buildAutoChooser();

                // Put the autoChooser on the SmartDashboard
                SmartDashboard.putData("Auto Chooser", autoChooser);
                // #endregion
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
                // #region Default Commands
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity());
                shooter.setDefaultCommand(shooter.stopCommand());
                extendo.setDefaultCommand(moveExtendoJoystick());
                intake.setDefaultCommand(intake.stopCommand());
                hopper.setDefaultCommand(hopper.stopCommand());
                // #endregion

                // #region Driver Controls
                driverXbox.leftBumper().whileTrue(lockDrive());
                driverXbox.leftTrigger().whileTrue(driveSlowCommand());
                driverXbox.rightTrigger().whileTrue(driveAimHub());

                driverXbox.start().whileTrue(zeroGyro());

                // #region Auto Path Commands
                @SuppressWarnings("unused")
                Command crossLeftTrenchToAlly = getPathfindThenFollowPathCommand("Cross Trench Left To Ally");
                @SuppressWarnings("unused")
                Command crossRightTrenchToAlly = getPathfindThenFollowPathCommand("Cross Trench Right To Ally");
                @SuppressWarnings("unused")
                Command crossLeftTrenchToNeutral = getPathfindThenFollowPathCommand("Cross Trench Left To Neutral");
                @SuppressWarnings("unused")
                Command crossRightTrenchToNeutral = getPathfindThenFollowPathCommand("Cross Trench Right To Neutral");

                // #endregion
                // #endregion

                // #region Operator Controls
                operatorXbox.leftBumper().whileTrue(agitateCommand());
                operatorXbox.rightTrigger()
                                .whileTrue(shootBlindSequenceCommand());
                operatorXbox.rightBumper()
                                .whileTrue(shootHubSequenceCommand());
                operatorXbox.leftTrigger()
                                .whileTrue(shootFerrySequenceCommand());
                operatorXbox.a().whileTrue(intakeAndSlowCommand());
                operatorXbox.b().whileTrue(intake.outCommand());
                operatorXbox.x().whileTrue(hopper.inCommand());
                operatorXbox.y().whileTrue(hopper.outCommand());
                // #endregion
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Pass in the selected auto from the SmartDashboard as our desired autnomous
                // commmand
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        private static Command getPathfindThenFollowPathCommand(String pathFileName) {
                PathConstraints constraints = new PathConstraints(
                                3.0, 4.0,
                                Units.degreesToRadians(540), Units.degreesToRadians(720));

                try {
                        PathPlannerPath path = PathPlannerPath
                                        .fromPathFile(pathFileName);

                        return AutoBuilder
                                        .pathfindThenFollowPath(path, constraints);
                } catch (FileVersionException | IOException | ParseException e) {
                        // TODO: At least log issue
                        return Commands.none();
                }
        }
}
