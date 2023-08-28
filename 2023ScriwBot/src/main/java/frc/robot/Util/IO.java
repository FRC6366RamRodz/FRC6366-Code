// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
    private static final XboxController operator = new XboxController(0);

    public static boolean getOpY() {
        return operator.getYButton();
    }

    public static boolean getOpX() {
        return operator.getXButton();
    }

    public static boolean getOpA() {
        return operator.getAButton();
    }

    public static boolean getOpB() {
        return operator.getBButton();
    }

    public static double joyDeadBand(double stickValue) {
        if (Math.abs(stickValue) <= 0.15) {
            return 0;
        } else {
            return stickValue;
        }
    }

    public static double responseCurve(double inputValue) {
        return inputValue*inputValue*inputValue;
    }

    public static double getLeftY() {
        return joyDeadBand(operator.getLeftY());
    }

    public static double getRightX() {
        return responseCurve(joyDeadBand(operator.getRightX()));
    }
}
