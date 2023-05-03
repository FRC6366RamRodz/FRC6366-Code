// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Util.Constants.DT_STG;

/** Add your docs here. */
public class IO {
    private static final XboxController driver = new XboxController(0);

    public static double joyDeadBand(double stickValue) {
        if (Math.abs(stickValue) <= DT_STG.DeadBand) {
            return 0;
        } else {
            return stickValue;
        }
    }

    public static double responseCurve(double inputValue) {
        return inputValue*inputValue*inputValue;
    }

    public static double getLeftY() {
        return joyDeadBand(driver.getLeftY());
    }

    public static double getRightX() {
        return responseCurve(joyDeadBand(driver.getRightX()));
    }

    public static boolean getAButton() {//we wont use this for now but its and example of a button being called.
        return driver.getAButton();
    }
}
