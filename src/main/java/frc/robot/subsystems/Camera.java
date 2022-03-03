// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for the cameras connected to the robot.
 * <p>
 * There is are two cameras: one on the front and one on the back.
 */
public class Camera extends SubsystemBase {
  UsbCamera frontCamera;
  UsbCamera backCamera;

  /** Creates a new Camera subsystem. */
  public Camera() {
    frontCamera = CameraServer.startAutomaticCapture("Front Camera", Constants.FRONT_CAMERA_CHANNEL);
    backCamera = CameraServer.startAutomaticCapture("Back Camera", Constants.BACK_CAMERA_CHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
