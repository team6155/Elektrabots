/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Add your docs here.
 */
public class DirectionalPad extends Button {
    private final Joystick JOYSTICK;
    private final int POV;

    public DirectionalPad(Joystick joystick, int pov) {
        this.JOYSTICK = joystick;
        this.POV = pov;
    }

    public boolean get() {
        return this.JOYSTICK.getPOV() == POV;
    }
}
