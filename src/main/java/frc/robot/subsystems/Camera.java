// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class Camera extends SubsystemBase {
  private final UsbCamera MAIN_CAMERA;
  private final UsbCamera GRABBER_CAMERA;

  /** Creates a new Camera. */
  public Camera() {
    MAIN_CAMERA = CameraServer.startAutomaticCapture("Main Camera", CameraConstants.MAIN_CAMERA_CHANNEL);
    GRABBER_CAMERA = CameraServer.startAutomaticCapture("Grabber Camera", CameraConstants.GRABBER_CAMERA_CHANNEL);

    MAIN_CAMERA.setResolution(640, 480);
    GRABBER_CAMERA.setResolution(640, 480);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
