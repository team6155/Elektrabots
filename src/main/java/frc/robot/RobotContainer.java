// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TestDrivingMotors;
import frc.robot.commands.TestRotationMotors;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TeleOpDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);
  private final ADXRS450_Gyro GYRO = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  // The robot's subsystems
  private final Drivetrain DRIVETRAIN_SUBSYSTEM = new Drivetrain(GYRO);
  private final Intake INTAKE_SUBSYSTEM = new Intake();
  private final Shooter SHOOTER_SUBSYSTEM = new Shooter();
  private final Arm ARM_SUBSYSTEM = new Arm();
  private final Camera CAMERA = new Camera();

  // The robot's commands
  private final TeleOpDrive DRIVE_COMMAND = new TeleOpDrive(
    DRIVETRAIN_SUBSYSTEM,
    () -> -driverController.getLeftY(),
    () -> -driverController.getLeftX(),
    () -> -driverController.getRightX(),
    () -> driverController.getLeftTriggerAxis() > .8,
    true,
    () -> driverController.getLeftTriggerAxis()
  );
  
  private final IntakeCommand intakecommand = new IntakeCommand(
    INTAKE_SUBSYSTEM,
    () -> operatorController.getLeftTriggerAxis(),
    () -> operatorController.leftBumper().getAsBoolean()
  );

  private final ShooterCommand shootercommand = new ShooterCommand(
    SHOOTER_SUBSYSTEM,
    () -> operatorController.getRightTriggerAxis(),
    () -> operatorController.leftBumper().getAsBoolean()
  );

  private final ArmCommand armCommand = new ArmCommand(ARM_SUBSYSTEM, () -> -operatorController.getLeftY());
  //private final Command testShooterAndIntake = Commands.parallel(intakecommand, shootercommand);
  private final TestDrivingMotors testDrivingMotors = new TestDrivingMotors(DRIVETRAIN_SUBSYSTEM);
  private final TestRotationMotors testRotationMotors = new TestRotationMotors(DRIVETRAIN_SUBSYSTEM);
  //private final Command testDrivetrain = testDrivingMotors.andThen(testRotationMotors);
  private final ResetGyro resetGyroCommand = new ResetGyro(GYRO);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVE_COMMAND);
    ARM_SUBSYSTEM.setDefaultCommand(armCommand);
    INTAKE_SUBSYSTEM.setDefaultCommand(intakecommand);
    SHOOTER_SUBSYSTEM.setDefaultCommand(shootercommand);

    // Configure the button bindings
    configureButtonBindings();

    GYRO.calibrate();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutonomousConstants.kMaxSpeedMetersPerSecond,
        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Empty list of "in-between" points for the robot to pass through.
        List.of(),
        // End 2 meter straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutonomousConstants.kPThetaController, 0, 0, AutonomousConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        DRIVETRAIN_SUBSYSTEM::getPose, // Functional interface to feed supplier
        DriveConstants.DRIVE_KINEMATICS,

        // Position controllers
        new PIDController(AutonomousConstants.kPXController, 0, 0),
        new PIDController(AutonomousConstants.kPYController, 0, 0),
        thetaController,
        DRIVETRAIN_SUBSYSTEM::setModuleStates,
        DRIVETRAIN_SUBSYSTEM);

    // Reset odometry to the starting pose of the trajectory.
    DRIVETRAIN_SUBSYSTEM.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return resetGyroCommand.andThen(swerveControllerCommand.andThen(() -> DRIVETRAIN_SUBSYSTEM.drive(0, 0, 0, false, false)));
  }

  public void resetGyro() {
    GYRO.reset();
  }

  public void calibrateGyro() {
    GYRO.calibrate();
  }
}
