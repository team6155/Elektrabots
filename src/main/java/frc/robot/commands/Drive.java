// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wheels;

public class Drive extends CommandBase {
  public Wheels wheels;
  public XboxController controller;
  
  /** Creates a new Drive. */
  public Drive(Wheels wheels) {
    this.wheels=wheels;
    addRequirements(wheels);
    controller=new XboxController(0);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = controller.getLeftY(); 
    double rotationalSpeed = controller.getRightX();
    wheels.frontLeftWheel.set(normalize(forwardSpeed + rotationalSpeed));
    wheels.rearLeftWheel.set(normalize(forwardSpeed + rotationalSpeed));
    wheels.frontRightWheel.set(normalize(forwardSpeed - rotationalSpeed));
    wheels.rearRightWheel.set(normalize(forwardSpeed - rotationalSpeed));



  }

  private double normalize(double speed) {
    if(speed>1||speed<1) {
     return speed/2;
    }
    return speed;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
