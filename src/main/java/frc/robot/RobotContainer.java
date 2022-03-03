// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.commands.AutonomousGroup;
import frc.robot.commands.ChangeRobotDirection;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.TeleOpDrive;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
  private final Gyro GYRO = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  // The robot's subsystems
  private final Drivetrain DRIVETRAIN_SUBSYSTEM = new Drivetrain(GYRO);
  private final Conveyor CONVEYOR_SUBSYSTEM = new Conveyor();
  private final Camera camera = new Camera();

  // The robot's commands
  private final TeleOpDrive DRIVE_COMMAND = new TeleOpDrive(DRIVETRAIN_SUBSYSTEM, driverController);
  private final RunConveyor CONVEYOR_COMMAND = new RunConveyor(CONVEYOR_SUBSYSTEM, operatorController);
  private final ChangeRobotDirection CHANGE_DIRECTION_COMMAND = new ChangeRobotDirection(DRIVETRAIN_SUBSYSTEM);
  private final AutonomousGroup AUTONOMOUS_COMMANDS = new AutonomousGroup(DRIVETRAIN_SUBSYSTEM, CONVEYOR_SUBSYSTEM);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    GYRO.calibrate();
    GYRO.reset();
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVE_COMMAND);
    CONVEYOR_SUBSYSTEM.setDefaultCommand(CONVEYOR_COMMAND);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton aButton = new JoystickButton(driverController, Constants.CONTROLLER_A_BUTTON);
    aButton.whenPressed(CHANGE_DIRECTION_COMMAND);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return AUTONOMOUS_COMMANDS;
  }
}
