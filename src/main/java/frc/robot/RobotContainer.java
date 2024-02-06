// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TeleOpDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  CommandJoystick operatorController = new CommandJoystick(Constants.OPERATOR_CONTROLLER_PORT);
  private final ADXRS450_Gyro GYRO = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  // The robot's subsystems
  private final Drivetrain DRIVETRAIN_SUBSYSTEM = new Drivetrain();

  // The robot's commands
  private final TeleOpDrive DRIVE_COMMAND = new TeleOpDrive(
    DRIVETRAIN_SUBSYSTEM,
    GYRO,
    () -> driverController.getLeftX(),
    () -> driverController.getLeftY(),
    () -> driverController.getRightX(),
    () -> driverController.getRightTriggerAxis() > .8,
    () -> driverController.getLeftTriggerAxis() > .8,
    true
  );


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVE_COMMAND);

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
    return null;
  }

  public void resetGyro() {
    GYRO.reset();
  }

  public void calibrateGyro() {
    GYRO.calibrate();
  }
}
