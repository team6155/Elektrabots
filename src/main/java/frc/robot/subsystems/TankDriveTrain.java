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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class TankDriveTrain extends DriveTrain {
  private SpeedController frontRightWheel;
  private SpeedController frontLeftWheel;
  private SpeedController middleRightWheel;
  private SpeedController middleLeftWheel;
  private SpeedController rearRightWheel;
  private SpeedController rearLeftWheel;
  private SpeedControllerGroup rightWheels;
  private SpeedControllerGroup leftWheels;

  public TankDriveTrain() {
    super();
    frontRightWheel = new PWMVictorSPX(RobotMap.FRONT_RIGHT_WHEEL);
    frontLeftWheel = new PWMVictorSPX(RobotMap.FRONT_LEFT_WHEEL);
    middleRightWheel = new PWMVictorSPX(RobotMap.MIDDLE_RIGHT_WHEEL);
    middleLeftWheel = new PWMVictorSPX(RobotMap.MIDDLE_LEFT_WHEEL);
    rearRightWheel = new PWMVictorSPX(RobotMap.REAR_RIGHT_WHEEL);
    rearLeftWheel = new PWMVictorSPX(RobotMap.REAR_LEFT_WHEEL);
    rightWheels = new SpeedControllerGroup(frontRightWheel, middleRightWheel, rearRightWheel);
    leftWheels = new SpeedControllerGroup(frontLeftWheel, middleLeftWheel, rearLeftWheel);
    wheels = new DifferentialDrive(rightWheels, leftWheels);
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
    ((DifferentialDrive) wheels).arcadeDrive(forwardsSpeed, rotationSpeed);
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
