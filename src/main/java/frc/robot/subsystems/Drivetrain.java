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
import frc.robot.commands.Drive;

public class Drivetrain extends PIDSubsystem {
  MecanumDrive mecanum;
  MotorController frontLeftWheel;
  MotorController backLeftWheel;
  MotorController frontRightWheel;
  MotorController backRightWheel;
  int direction;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    frontLeftWheel = new PWMVictorSPX(Constants.FRONT_LEFT_WHEEL_CHANNEL);
    backLeftWheel = new PWMVictorSPX(Constants.BACK_LEFT_WHEEL_CHANNEL);
    frontRightWheel = new PWMVictorSPX(Constants.FRONT_RIGHT_WHEEL_CHANNEL);
    backRightWheel = new PWMVictorSPX(Constants.BACK_RIGHT_WHEEL_CHANNEL);
    mecanum = new MecanumDrive(frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel);
    direction = 1;
  }

  public void drive(double ySpeed, double xSpeed, double zRotation) {
    mecanum.driveCartesian(ySpeed * direction, xSpeed * direction, zRotation);
  }

  public int changeDirection() {
    direction = -direction;
    return direction;
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
