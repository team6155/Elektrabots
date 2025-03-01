// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.Drivetarget;
import frc.robot.commands.Seektarget;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final SwerveDrive swerveDrive;
  private final Elevator elevator;
  private final Intake intake;
  private final Hanger hanger;
  private final Seektarget seekTarget;
  private final Vision m_Limelight;
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort
  );
  private final CommandXboxController m_operatorController =
    new CommandXboxController(OperatorConstants.kOperatorControllerPort
    );
  private final SwerveDrivePoseEstimator poseEstimator;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    poseEstimator = new SwerveDrivePoseEstimator(
      SwerveModuleConstants.driveKinematics,
      new Rotation2d(),
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      },
      new Pose2d(),
      VecBuilder.fill(.05, .05, Units.degreesToRadians(5)),
      VecBuilder.fill(.7, .7, 9999999)
    );

    swerveDrive = new SwerveDrive(poseEstimator);
    elevator = new Elevator();
    intake = new Intake();
    hanger = new Hanger();
    m_Limelight = new Vision(poseEstimator);

    
    seekTarget = new Seektarget(m_Limelight, swerveDrive);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    swerveDrive.setDefaultCommand(
      new RunCommand(
        () -> swerveDrive.Drive(
          MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.deadbandValue), 
          MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.deadbandValue), 
          MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.deadbandValue)
        )
      )
    );
    hanger.setDefaultCommand(
      new RunCommand(
        () -> hanger.run(
          m_driverController.getLeftTriggerAxis(),
          m_driverController.getRightTriggerAxis()
        )
      )
    );
    elevator.setDefaultCommand(
      new RunCommand(
        () ->  elevator.run(
          m_operatorController.getLeftY()
        )
      )
    );
    intake.setDefaultCommand(
      new RunCommand(
        () -> intake.run(
          m_operatorController.getRightY()
        )
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return seekTarget;
  }
}
