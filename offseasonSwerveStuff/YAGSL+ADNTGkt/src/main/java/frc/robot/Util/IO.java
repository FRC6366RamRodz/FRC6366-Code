// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
    private static final XboxController driver = new XboxController(0);

    public static double getLeftY() {
        return -driver.getLeftY();
    }

    public static double getLeftX() {
        return -driver.getLeftX();
    }

    public static double getRightY() {
        return -driver.getRightY();
    }

    public static double getRightX() {
        return -driver.getRightX();
    }

    public static boolean getYButton() {
        return driver.getYButton();
    }

    public static boolean getYButtonReleased() {
        return driver.getYButtonReleased();
    }

    public static boolean getYbuttonPressed() {
        return driver.getYButtonPressed();
    }
}
