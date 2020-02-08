/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;

/**
 * Subsystem for controlling the robot's wheels for a mecanum setup with four wheels.
 */
public class MecanumDriveTrain extends DriveTrain {
  private SpeedController frontRightWheel;
  private SpeedController frontLeftWheel;
  private SpeedController rearRightWheel;
  private SpeedController rearLeftWheel;

  public MecanumDriveTrain() {
    super();
    frontRightWheel = new PWMVictorSPX(RobotMap.FRONT_RIGHT_WHEEL);
    frontLeftWheel = new PWMVictorSPX(RobotMap.FRONT_LEFT_WHEEL);
    rearRightWheel = new PWMVictorSPX(RobotMap.REAR_RIGHT_WHEEL);
    rearLeftWheel = new PWMVictorSPX(RobotMap.REAR_LEFT_WHEEL);
    wheels = new MecanumDrive(frontLeftWheel, rearLeftWheel, frontRightWheel, rearRightWheel);
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
    drive(forwardsSpeed, 0, rotationSpeed);
  }

  /**
   * Drive the robot according to the given inputs.
   * 
   * @param forwardsSpeed The robot's speed forwards or backwards [-1.0..1.0].
   *                      Back is negative and forwards is positive.
   * @param sidewaysSpeed The robot's speed left or right [-1.0..1.0].
   *                      Left is negative and right is positive.
   * @param rotationSpeed The robot's rotational speed [-1.0..1.0].
   *                      Counter-clockwise is negative and clockwise is positive.
   */
  @Override
  public void drive(double forwardsSpeed, double sidewaysSpeed, double rotationSpeed) {
    ((MecanumDrive) wheels).driveCartesian(forwardsSpeed, sidewaysSpeed, rotationSpeed);
  }
}
