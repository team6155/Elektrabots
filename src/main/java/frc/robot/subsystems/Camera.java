// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase {
  private final UsbCamera INTAKE_CAMERA;
  private final UsbCamera SHOOTER_CAMERA;

  /** Creates a new Camera. */
  public Camera() {
    INTAKE_CAMERA = CameraServer.startAutomaticCapture("Intake Camera", Constants.INTAKE_CAMERA_CHANNEL);
    INTAKE_CAMERA.setResolution(320, 240);
    INTAKE_CAMERA.setFPS(15);

    SHOOTER_CAMERA = CameraServer.startAutomaticCapture("Shooter Camera", Constants.SHOOTER_CAMERA_CHANNEL);
    SHOOTER_CAMERA.setResolution(320, 240);
    SHOOTER_CAMERA.setFPS(15);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
