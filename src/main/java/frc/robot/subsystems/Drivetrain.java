// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
  Gyro gyro;
  int direction;
  double rotationMultiple;

  double forwardsSpeed;
  double sidewaysSpeed;

  /** Creates a new Drivetrain subsystem. */
  public Drivetrain() {
    super(new PIDController(1, 0, 0));
    frontLeftWheel = new PWMVictorSPX(Constants.FRONT_LEFT_WHEEL_CHANNEL);
    backLeftWheel = new PWMVictorSPX(Constants.BACK_LEFT_WHEEL_CHANNEL);
    frontRightWheel = new PWMVictorSPX(Constants.FRONT_RIGHT_WHEEL_CHANNEL);
    backRightWheel = new PWMVictorSPX(Constants.BACK_RIGHT_WHEEL_CHANNEL);
    frontRightWheel.setInverted(true);
    backRightWheel.setInverted(true);
    // The right and left sides are reversed because of the wheel installation.
    mecanum = new MecanumDrive(frontRightWheel, backRightWheel, frontLeftWheel, backLeftWheel);
    gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    direction = 1;
    rotationMultiple = 1;
    enable();
  }

  /**
   * Drive the robot according to the given inputs.
   * @param forwardsSpeed The desired speed forwards. [-1.0..1.0]. Positive is forwards, negative is backwards.
   * @param sidewaysSpeed The desired speed sideways. [-1.0..1.0]. Positive is right, negative is left.
   * @param rotationalSpeed The desired rotational speed. [-1.0..1.0]. Positive is clockwise, negative is counter-clockwise.
   */
  public void drive(double forwardsSpeed, double sidewaysSpeed, double rotationalSpeed) {
    this.forwardsSpeed = forwardsSpeed;
    this.sidewaysSpeed = sidewaysSpeed;
    setSetpoint(rotationalSpeed * rotationMultiple);
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
    mecanum.driveCartesian(forwardsSpeed * direction, sidewaysSpeed * direction, output);
  }

  @Override
  public double getMeasurement() {
    return gyro.getRate();
  }
}
