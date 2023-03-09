// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lights;

public class ControlLights extends CommandBase {
  private Lights lights;
  private Drivetrain drivetrain;
  private double red;
  private double green;
  private double blue;
  private double safetyTime;

  /** Creates a new ControlLights. */
  public ControlLights(Lights lights, Drivetrain drivetrain) {
    this.lights = lights;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = lights.getTime() / LEDConstants.COLOR_FREQUENCY;
    boolean increasing = (int)time % 2 == 0;
    if ((int)time % 3 == 0) {
      if (increasing) {
        red = time % 1;
      }
      else {
        red = 1 - (time % 1);
      }
    }
    if ((int)time % 3 == 1) {
      if (increasing) {
        green = time % 1;
      }
      else {
        green = 1 - (time % 1);
      }      }
    if ((int)time % 3 == 2) {
      if (increasing) {
        blue = time % 1;
      }
      else {
        blue = 1 - (time % 1);
      }      
    }
    if (drivetrain.idle) {
      if (lights.getTime() - safetyTime > 0.5) {
        lights.breathe(red, green, blue);
      }
      else {
        lights.blink();
      }
    }
    else {
      safetyTime = lights.getTime();
      lights.blink();
    }
    SmartDashboard.putNumber("R", red);
    SmartDashboard.putNumber("G", green);
    SmartDashboard.putNumber("B", blue);
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
