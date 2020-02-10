package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.ChangeDirection;
import frc.robot.commands.ExtendPusher;
import frc.robot.commands.RetractPusher;
import frc.robot.commands.UpdateSensors;
import frc.robot.controllers.Gamepad;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public final Gamepad DRIVER_CONTROLLER;
  public final Gamepad OPERATOR_CONTROLLER;

  private final Button DRIVER_A_BUTTON;
  private final Button DRIVER_B_BUTTON;
  private final Button OPERATOR_RIGHT_BUMPER;
  private final Button OPERATOR_LEFT_BUMPER;

  /**
   * Constructor for the OI class.
   */
  public OI() {
    DRIVER_CONTROLLER = new Gamepad(RobotMap.DRIVER_GAMEPAD);
    DRIVER_A_BUTTON = DRIVER_CONTROLLER.A_BUTTON;
    DRIVER_B_BUTTON = DRIVER_CONTROLLER.B_BUTTON;

    DRIVER_A_BUTTON.whenPressed(new ChangeDirection());
    DRIVER_B_BUTTON.whenPressed(new UpdateSensors());

    OPERATOR_CONTROLLER = new Gamepad(RobotMap.OPERATOR_GAMEPAD);
    OPERATOR_RIGHT_BUMPER = OPERATOR_CONTROLLER.RIGHT_BUMPER;
    OPERATOR_LEFT_BUMPER = OPERATOR_CONTROLLER.LEFT_BUMPER;

    OPERATOR_RIGHT_BUMPER.whenPressed(new ExtendPusher());
    OPERATOR_LEFT_BUMPER.whenPressed(new RetractPusher());
  }
}
