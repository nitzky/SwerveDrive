// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();

        private final IntakeSubsystem m_intake = new IntakeSubsystem();

        private final ShooterSubsystem m_shooter = new ShooterSubsystem();

        // Controller Buttons Used

        // **********************************************************
        // Xbox Controller - 1/Driver - Port 0
        // ----------------------------------------------------------
        // Left Stick (Translate along X and Y plane) - "Swerve"
        // Right Stick (Rotate about Z-Axis) - Drive Right/Left
        //
        // **********************************************************

        // **********************************************************
        // Xbox Controller - 2/Operator - Port 1
        // ----------------------------------------------------------
        // Left Trigger -
        // Right Trigger -
        // Button A - Intake in
        // Button B -
        // Button X -
        // Button Y -
        // **********************************************************

        // The driver's controller
        XboxController m_driverController = new XboxController(
                        OIConstants.kDriverControllerPort);

        // The operator's controller
        XboxController m_operatorController = new XboxController(
                        OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband), // * 0.95
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband), // * 0.95
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband), // * 0.95
                                                                true,
                                                                true),
                                                m_robotDrive));

                // Build an auto chooser. This will use Commands.none() as the default option.

                PathPlannerPath leftCurl = PathPlannerPath.fromPathFile("Left Curl");
                PathPlannerPath amp = PathPlannerPath.fromPathFile("Amp");
                PathPlannerPath spk_r1_spk = PathPlannerPath.fromPathFile("spk_r1_spk");

                pathPlannerChooser.addOption("Left Curl", leftCurl);
                pathPlannerChooser.addOption("Amp", amp);
                pathPlannerChooser.addOption("spk_r1_spk", spk_r1_spk);
                Shuffleboard.getTab("PathPlanner Autonomous").add(pathPlannerChooser);
        }

        private SendableChooser<PathPlannerPath> pathPlannerChooser = new SendableChooser<>();

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                // Lock wheels in 'X' pattern
                new JoystickButton(m_driverController, Button.kLeftBumper.value)
                                .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

                // Ring Intake In on floor
                new JoystickButton(m_operatorController, Button.kRightBumper.value)
                                .whileTrue(new RunCommand(() -> m_intake.startIntake(), m_intake))
                                .whileFalse(new RunCommand(() -> m_intake.stopIntake(), m_intake));
                // Reverse Intake
                new JoystickButton(m_operatorController, Button.kLeftBumper.value)
                                .whileTrue(new RunCommand(() -> m_intake.ReverseIntake(), m_intake))
                                .whileFalse(new RunCommand(() -> m_intake.stopIntake(), m_intake));

                // Shoot ring into speaker (pew pew!)
                new JoystickButton(m_operatorController, Button.kA.value)
                                .whileTrue(new RunCommand(() -> m_shooter.startShooter(), m_shooter))
                                .whileFalse(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // An example trajectory to follow. All units in meters.
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // config);
                //
                // var thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // m_robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,

                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // m_robotDrive::setModuleStates,
                // m_robotDrive);

                // // // Reset odometry to the starting pose of the trajectory.
                // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
                // false, false));

                // Load the path you want to follow using its name in the GUI
                // PathPlannerPath path = PathPlannerPath.fromPathFile("Path");

                // Create a path following command using AutoBuilder. This will also trigger
                // event markers.

                // Resets the robotOdometry to the currently selected path in the chooser
                // If blue, odometry will be reset to starting pose of the path
                // Else if red, odometry will be reset to the flipped starting pose of the blue
                // path
                m_robotDrive.resetOdometry(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                                ? pathPlannerChooser.getSelected().getPreviewStartingHolonomicPose()
                                : pathPlannerChooser.getSelected().flipPath().getPreviewStartingHolonomicPose());
                ;
                return AutoBuilder.followPath(pathPlannerChooser.getSelected());
        }
}
