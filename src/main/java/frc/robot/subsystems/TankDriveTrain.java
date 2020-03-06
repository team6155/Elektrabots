/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.utility.Direction;

/**
 * Subsystem for controlling the robot's wheels for a mecanum setup with four wheels.
 */
public class TankDriveTrain extends PIDDrive {
  public SpeedController frontRightWheel;
  public SpeedController rearRightWheel;
  public SpeedController frontLeftWheel;
  public SpeedController rearLeftWheel;
  private SpeedControllerGroup rightWheels;
  private SpeedControllerGroup leftWheels;

  public TankDriveTrain() {
    super();
    frontRightWheel = new PWMVictorSPX(RobotMap.FRONT_RIGHT_WHEEL);
    rearRightWheel = new PWMVictorSPX(RobotMap.REAR_RIGHT_WHEEL);
    frontLeftWheel = new PWMVictorSPX(RobotMap.FRONT_LEFT_WHEEL);
    rearLeftWheel = new PWMVictorSPX(RobotMap.REAR_LEFT_WHEEL);
    rightWheels = new SpeedControllerGroup(frontRightWheel, rearRightWheel);
    leftWheels = new SpeedControllerGroup(frontLeftWheel, rearLeftWheel);
    rightWheels.setInverted(true);
    leftWheels.setInverted(true);
    wheels = new DifferentialDrive(leftWheels, rightWheels);
    //enable();
  }

  /**
   * Drive the robot according to the given inputs.
   * 
   * @param leftSpeed The speed of the robot's left wheels. [-1.0..1.0].
   *                      Back is negative and forwards is positive.
   * @param rightSpeed The speed of the robot's right wheels. [-1.0..1.0].
   *                      Back is negative and forwards is positive.
   */
  @Override
  public void drive(double leftSpeed, double rightSpeed) {
    if(direction == Direction.FORWARD) {
      ((DifferentialDrive) wheels).tankDrive(direction * leftSpeed, direction * rightSpeed, false);
    }
    else {
      ((DifferentialDrive) wheels).tankDrive(direction * rightSpeed, direction * leftSpeed, false);
    }
  }

  @Override
  protected void usePIDOutput(double output) {
    // TODO Auto-generated method stub
    double leftSpeed = leftWheels.get() + output;
    double rightSpeed = rightWheels.get() - output;

    double normalizeValue = normalizeSpeed(leftSpeed);
    leftSpeed -= normalizeValue;
    rightSpeed += normalizeValue;

    normalizeValue = normalizeSpeed(rightSpeed);
    leftSpeed += normalizeValue;
    rightSpeed -= normalizeValue;

    drive(leftSpeed, rightSpeed);
  }

  private double normalizeSpeed(double speed) {
    if(speed > 1){
      return speed - 1;
    }
    if(speed < -1){
      return speed + 1;
    }
    return 0;
  }
}
