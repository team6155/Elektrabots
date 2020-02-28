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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * Subsystem for controlling the robot's wheels for a mecanum setup with four wheels.
 */
public class TankDriveTrain extends DriveTrain {
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
  }

  /**
   * Drive the robot according to the given inputs.
   * 
   * @param forwardsSpeed The robot's speed forwards or backwards [-1.0..1.0].
   *                      Back is negative and forwards is positive.
   * @param rotationSpeed The robot's rotational speed [-1.0..1.0].
   *                      Counter-clockwise is negative and clockwise is positive.
   */
  @Override
  public void drive(double forwardsSpeed, double rotationSpeed) {
    ((DifferentialDrive) wheels).arcadeDrive(direction * forwardsSpeed, rotationSpeed);
  }

  /**
   * Drive the robot according to the given inputs.
   * 
   * @param forwardsSpeed The robot's speed forwards or backwards [-1.0..1.0].
   *                      Back is negative and forwards is positive.
   * @param sidewaysSpeed UNUSED. A robot with a Tank Drive setup cannot drive sideways.
   * @param rotationSpeed The robot's rotational speed [-1.0..1.0].
   *                      Counter-clockwise is negative and clockwise is positive.
   */
  @Override
  public void drive(double forwardsSpeed, double sidewaysSpeed, double rotationSpeed) {
    drive(forwardsSpeed, rotationSpeed);
  }
}
