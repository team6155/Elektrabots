package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.ChangeDirection;
import frc.robot.commands.ExtendPusher;
import frc.robot.commands.RetractPusher;
import frc.robot.controllers.Gamepad;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private final Gamepad CONTROLLER;
  private final Button RIGHT_BUMPER;
  private final Button LEFT_BUMPER;
  private final Button A_BUTTON;

  /**
   * Constructor for the OI class.
   */
  public OI() {
    CONTROLLER = new Gamepad(RobotMap.GAMEPAD);
    RIGHT_BUMPER = CONTROLLER.RIGHT_BUMPER;
    LEFT_BUMPER = CONTROLLER.LEFT_BUMPER;
    A_BUTTON = CONTROLLER.A_BUTTON;

    RIGHT_BUMPER.whenPressed(new ExtendPusher());
    LEFT_BUMPER.whenPressed(new RetractPusher());
    A_BUTTON.whenPressed(new ChangeDirection());
  }

  /**
   * Get the xbox controller.
   * 
   * @return The xbox controller.
   */
  public Gamepad getController() {
    return CONTROLLER;
  }
}
