package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands
 * and command groups that allow control of the robot.
 */
public class OI {
  private XboxController controller;

  /**
   * Constructor for the OI class.
   */
  public OI() {
    controller = new XboxController(RobotMap.XBOX_CONTROLLER);
  }

  /**
   * Get the xbox controller.
   * 
   * @return The xbox controller.
   */
  public XboxController getController() {
    return controller;
  }
}
