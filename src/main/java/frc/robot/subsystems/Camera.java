package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Subsystem controlling the cameras of the robot.
 * <p>
 * The robot currently has two cameras, one on each side of the robot. Only one
 * can be shown at a time, so the user must be able to switch between them
 * easily.
 */
public class Camera extends Subsystem {
  UsbCamera frontCamera;
  UsbCamera backCamera;
  VideoSink server;

  /**
   * Constructor for the Camera subsystem.
   */
  public Camera() {
    // Create both cameras as well as the server the video will be sent to for
    // display.
    frontCamera = CameraServer.getInstance().startAutomaticCapture(RobotMap.FRONT_CAMERA);
    backCamera = CameraServer.getInstance().startAutomaticCapture(RobotMap.BACK_CAMERA);
    server = CameraServer.getInstance().getServer();

    // Set the resolution of both cameras.
    frontCamera.setResolution(640, 480);
    backCamera.setResolution(640, 480);

    // By default, the camera not in use will be disconnected, leading to a delay
    // when it is switched to. By telling the system to keep both cameras open, this
    // can be avoided.
    // Keep in mind that there is a limit to how much video can be taken at a time,
    // so with more cameras at higher resolution, this can cause problems.
    //frontCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    //backCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    setCamera(frontCamera.getName());
  }

  /**
   * Switch the active camera.
   * <p>
   * By default, only one camera can be active at a time. The intention of this
   * method is that the user can switch between the two cameras at will.
   */
  public void changeDirection() {
    // Get the currently active camera then switch to the inactive camera.
    String currentCamera = getCamera();
    currentCamera = currentCamera.equals(frontCamera.getName()) ? backCamera.getName() : frontCamera.getName();
    setCamera(currentCamera);
  }

  /**
   * Set the given camera as the active camera.
   * 
   * @param camera Camera to switch to.
   */
  private void setCamera(String camera) {
    NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection").setString(camera);
  }

  private String getCamera() {
    return NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection").getString("");
  }

  /**
   * Initialize the default command for a subsystem. By default subsystems have no
   * default command, but if they do, the default command is set with this method.
   * It is called on all Subsystems by CommandBase in the users program after all
   * the Subsystems are created.
   * <p>
   * When the subsystem is not currently in use, it will call the command set in
   * this method.
   */
  @Override
  public void initDefaultCommand() {
    // The camera subsystem currently needs no default command.
  }
}
