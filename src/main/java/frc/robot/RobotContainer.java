// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autonomous;
import frc.robot.commands.ChangeRobotDirection;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.TeleOpDrive;
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

  // The robot's subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Conveyor conveyor = new Conveyor();
  // TODO: Activate camera subsytem once the cameras have been attached to the robot.
  // private final Camera camera = new Camera();

  // The robot's commands
  private final TeleOpDrive drive_command = new TeleOpDrive(drivetrain, driverController);
  private final RunConveyor conveyor_command = new RunConveyor(conveyor, operatorController);
  private final ChangeRobotDirection change_direction = new ChangeRobotDirection(drivetrain);
  private final Autonomous autonomous = new Autonomous(drivetrain, conveyor);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(drive_command);
    conveyor.setDefaultCommand(conveyor_command);
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
    aButton.whenPressed(change_direction);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomous;
  }
}
