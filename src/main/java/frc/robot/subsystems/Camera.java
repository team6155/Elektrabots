/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

// TODO: Documentation
/**
 * Add your docs here.
 */
public class Camera extends Subsystem {
  UsbCamera frontCamera;
  UsbCamera backCamera;
  VideoSink server;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Camera() {
    frontCamera = CameraServer.getInstance().startAutomaticCapture(RobotMap.FRONT_CAMERA);
    backCamera = CameraServer.getInstance().startAutomaticCapture(RobotMap.BACK_CAMERA);
    server = CameraServer.getInstance().getServer();

    frontCamera.setResolution(640, 480);
    backCamera.setResolution(640, 480);
    frontCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    backCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    setCamera(frontCamera);
  }

  public void changeDirection() {
    VideoSource currentCamera = server.getSource();
    currentCamera = currentCamera.equals(frontCamera) ? backCamera : frontCamera;
    setCamera(currentCamera);
  }

  private void setCamera(VideoSource camera) {
    server.setSource(camera);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
