// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
    private static final XboxController driver = new XboxController(0);


    public static double joyDeadband(double stickValue) {
        if (stickValue <= Constants.DT_Set.DT_DEADBAND && stickValue >= -Constants.DT_Set.DT_DEADBAND){
            return 0;
        } else {
            return stickValue;
        }
    }

    public static double getLeftY() {
        return joyDeadband(driver.getLeftY());
    }

    public static double getLeftX() {
        return joyDeadband(driver.getLeftX());
    }

    public static double getRightX() {
        return Math.pow(joyDeadband(driver.getRightX()), 2) * Math.signum(driver.getRightX());
    }

    public static double getStrafeSteer() {
        if (driver.getLeftTriggerAxis() >= driver.getRightTriggerAxis()) {
            return -joyDeadband(driver.getLeftTriggerAxis());
        } else {
            return joyDeadband(driver.getRightTriggerAxis());
        }
    }

    

    public static boolean getA() {
        boolean toggle = false;

        if (driver.getAButtonPressed() == true) {
            if (toggle) {
                toggle = false;
            } else {
                toggle = true;
            }
        }

        if (toggle == true){
            return true;
        } else {
            return false;
        }
    }

    public static boolean getLeftBumper() {
        boolean toggle = false;

        if (driver.getLeftBumperPressed() == true) {
            if (toggle) {
                toggle = false;
            } else {
                toggle = true;
            }
        }

        if (toggle == true){
            return true;
        } else {
            return false;
        }
    }

    public static boolean getRightBumper() {
        boolean toggle = false;

        if (driver.getRightBumperPressed() == true) {
            if (toggle) {
                toggle = false;
            } else {
                toggle = true;
            }
        }

        if (toggle == true){
            return true;
        } else {
            return false;
        }
    }
}
