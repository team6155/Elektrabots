// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

/** Subsystem of the robot for controlling driving. */
public class Drivetrain extends PIDSubsystem {
  MecanumDrive mecanum;
  MotorController frontLeftWheel;
  MotorController backLeftWheel;
  MotorController frontRightWheel;
  MotorController backRightWheel;
  int direction;

  /** Creates a new Drivetrain subsystem. */
  public Drivetrain() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    // TODO: Robot's wheels are not set up as the MecanumDrive class expects. May need to switch left and right sides.
    frontLeftWheel = new PWMVictorSPX(Constants.FRONT_LEFT_WHEEL_CHANNEL);
    backLeftWheel = new PWMVictorSPX(Constants.BACK_LEFT_WHEEL_CHANNEL);
    frontRightWheel = new PWMVictorSPX(Constants.FRONT_RIGHT_WHEEL_CHANNEL);
    backRightWheel = new PWMVictorSPX(Constants.BACK_RIGHT_WHEEL_CHANNEL);
    mecanum = new MecanumDrive(frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel);
    direction = 1;
  }

  /**
   * Drive the robot according to the given inputs.
   * @param forwardsSpeed The desired speed forwards. [-1.0..1.0]. Positive is forwards, negative is backwards.
   * @param sidewaysSpeed The desired speed sideways. [-1.0..1.0]. Positive is right, negative is left.
   * @param rotationalSpeed The desired rotational speed. [-1.0..1.0]. Positive is clockwise, negative is counter-clockwise.
   */
  public void drive(double forwardsSpeed, double sidewaysSpeed, double rotationalSpeed) {
    mecanum.driveCartesian(forwardsSpeed * direction, sidewaysSpeed * direction, rotationalSpeed);
  }

  /**
   * Switch the facing of the robot. This will change what side of the robot is considered to be the front.
   * @return
   */
  public void changeDirection() {
    direction = -direction;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
