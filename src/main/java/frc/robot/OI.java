package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.ExtendPusher;
import frc.robot.commands.RetractPusher;
import frc.robot.controllers.Gamepad;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private final Gamepad controller;
  private final Button rightbumper;
  private final Button leftbumper; 

  /**
   * Constructor for the OI class.
   */
  public OI() {
    controller = new Gamepad(RobotMap.GAMEPAD);
    rightbumper = controller.RIGHT_BUMPER;
    leftbumper = controller.LEFT_BUMPER;

    rightbumper.whenPressed(new ExtendPusher());
    leftbumper.whenPressed(new RetractPusher());
  }

  /**
   * Get the xbox controller.
   * 
   * @return The xbox controller.
   */
  public Gamepad getController() {
    return controller;
  }
}
