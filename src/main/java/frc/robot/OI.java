package frc.robot;

import frc.robot.commands.ChangeDirection;
import frc.robot.commands.ExtendHook;
import frc.robot.commands.ExtendLift;
import frc.robot.commands.ExtendScoop;
import frc.robot.commands.PositionControl;
import frc.robot.commands.RetractHook;
import frc.robot.commands.RetractLift;
import frc.robot.commands.RetractScoop;
import frc.robot.commands.RotationControl;
import frc.robot.controllers.Gamepad;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public final Gamepad DRIVER_CONTROLLER;
  public final Gamepad OPERATOR_CONTROLLER;

  /**
   * Constructor for the OI class.
   */
  public OI() {
    DRIVER_CONTROLLER = new Gamepad(RobotMap.DRIVER_GAMEPAD);
    OPERATOR_CONTROLLER = new Gamepad(RobotMap.OPERATOR_GAMEPAD);

    DRIVER_CONTROLLER.A_BUTTON.whenPressed(new ChangeDirection());
    DRIVER_CONTROLLER.RIGHT_BUMPER.whenPressed(new PositionControl());
    DRIVER_CONTROLLER.LEFT_BUMPER.whenPressed(new RotationControl());

    OPERATOR_CONTROLLER.A_BUTTON.whenPressed(new ExtendScoop());
    OPERATOR_CONTROLLER.B_BUTTON.whenPressed(new RetractScoop());
    OPERATOR_CONTROLLER.Y_BUTTON.whenPressed(new ExtendHook());
    OPERATOR_CONTROLLER.X_BUTTON.whenPressed(new RetractHook());
    OPERATOR_CONTROLLER.RIGHT_BUMPER.whenPressed(new ExtendLift());
    OPERATOR_CONTROLLER.LEFT_BUMPER.whenPressed(new RetractLift());
  }
}
