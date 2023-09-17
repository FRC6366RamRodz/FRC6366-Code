// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
    private static final XboxController driver = new XboxController(0);

    public static double getLeftY() {
        return driver.getLeftY();
    }

    public static double getLeftX() {
        return driver.getLeftX();
    }

    public static double getRightX() {
        return driver.getRightX();
    }

    public static double getRightTrigger() {
        return driver.getRightTriggerAxis();
    }

    public static double getLeftTrigger() {
        return driver.getLeftTriggerAxis();
    }

    public static boolean getLeftBumperPressed() {
        return driver.getLeftBumperPressed();
    }

    public static boolean getRightBumperPressed() {
        return driver.getRightBumperPressed();
    }

    public static boolean getXButtonPressed() {
        return driver.getXButtonPressed();
    }

    public static boolean getAbuttonPressed() {
        return driver.getAButtonPressed();
    }
}
