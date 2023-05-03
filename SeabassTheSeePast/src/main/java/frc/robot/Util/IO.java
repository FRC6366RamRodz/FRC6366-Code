// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Util.Constants.DT_STG;

/** Add your docs here. */
public class IO {
    private static final XboxController driver = new XboxController(0);

    public static double joyDeadband(double stickValue) {
        if (Math.abs(stickValue) <= DT_STG.Deadband) {
            return 0;
        } else {
            return stickValue;
        }
    }

    public static double cubedController(double inputValue) {
        return inputValue*inputValue*inputValue;
    }

    public static double getLeftY() {
        return joyDeadband(driver.getLeftX());
    }

    public static double getLeftX() {
        return joyDeadband(driver.getLeftX());
    }

    public static double getRightX() {
        return cubedController(joyDeadband(driver.getRightX()));
    }

    public static boolean getLeftTrigger() {
        if (driver.getLeftTriggerAxis() > 0.4) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean getLeftBumperPressed() {
        return driver.getLeftBumperPressed();
    }

    public static boolean getRightBumperPressed() {
        return driver.getRightBumperPressed();
    }

    public static boolean getBButton() {
        return driver.getBButton();
    }

    public static boolean getYButton() {
        return driver.getYButton();
    }
}
