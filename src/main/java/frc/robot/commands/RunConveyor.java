// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

/** Run the two conveyor motors according to controller inputs. */
public class RunConveyor extends CommandBase {
  Conveyor conveyor;
  XboxController controller;

  /**
   * Create a new RunConveyor command.
   * @param conveyor The robot's conveyor subsystem.
   * @param controller The xbox controller for controlling the conveyor.
   */
  public RunConveyor(Conveyor conveyor, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.conveyor = conveyor;
    this.controller = controller;
    addRequirements(conveyor);
  }
  
  
  @Override
  public void initialize() {
    
  }

  public void execute() {
    conveyor.runIntakeMotor(controller.getLeftY());
    conveyor.runShootingMotor(controller.getRightY());
  }

  public void end(boolean interrupted) {

  } 

  public boolean isFinished() {
    return false;
  }
}
