/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;
import frc.robot.utility.Direction;

/**
 * Add your docs here.
 */
public abstract class PIDDrive extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  protected RobotDriveBase wheels;
  protected double direction;
  protected Gyro gyro;
  
  public PIDDrive() {
    // Intert a subsystem name and PID values here
    super("DriveTrain", 1, 0, 0);
    direction = Direction.FORWARD;
    gyro = new ADXRS450_Gyro(RobotMap.GYRO);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  public double getDirection() {
      return direction;
  }

  public double readGyro() {
      return gyro.getAngle();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Drive(Robot.oi.DRIVER_CONTROLLER));
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return gyro.getAngle();
  }
  
  public abstract void drive(double leftSpeed, double rightSpeed);

  @Override
  protected abstract void usePIDOutput(double output);

  /**
   * Change the forwards direction of the robot.
   * 
   * @return The new direction of the robot.
   */
  public double changeDirection() {
      if (direction == Direction.FORWARD) {
          direction = Direction.REVERSE;
      }
      else {
          direction = Direction.FORWARD;
      }
      return direction;
  }
}
