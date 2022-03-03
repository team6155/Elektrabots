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
  private final UsbCamera FRONT_CAMERA;
  private final UsbCamera BACK_CAMERA;

  /** Creates a new Camera subsystem. */
  public Camera() {
    FRONT_CAMERA = CameraServer.startAutomaticCapture("Front Camera", Constants.FRONT_CAMERA_CHANNEL);
    BACK_CAMERA = CameraServer.startAutomaticCapture("Back Camera", Constants.BACK_CAMERA_CHANNEL);
    FRONT_CAMERA.setResolution(640, 480);
    BACK_CAMERA.setResolution(640, 480);
    FRONT_CAMERA.setFPS(30);
    BACK_CAMERA.setFPS(30);
  }
}
