// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
    private static final XboxController driver = new XboxController(0); //reference 360 for controll names

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

    public static double getRightTrigger() {//slowModer
        return 1-driver.getRightTriggerAxis()*ControllConstants.peakSlowMode;
    }

    public static boolean getBackPressed() { //trim left
        return driver.getBackButtonPressed();
    }

    public static boolean getStartPressed() { //trim right
        return driver.getStartButtonPressed();
    }

    public static boolean getPanic() {
        boolean panic;
        if (driver.getStartButton()==true && driver.getBackButton()==true) {
            panic = true;
        } else {
            panic = false;
        }

        return panic;
    }

    public static boolean getRightBumperPressed() {//autoDrive
       return driver.getRightBumperPressed();
    }

    public static boolean getRightBumperReleased() {//deactivate autoDrive
        return driver.getRightBumperReleased();
    }

    public static boolean getYbutton() {//lockPose
        return driver.getYButton();
    }

    public static boolean getLeftTrigger() {
        if (driver.getLeftTriggerAxis() > 0.15) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean getLeftBumper() {
        return driver.getLeftBumper();
    }

    //Operator
    private static final XboxController operator = new XboxController(1); //reference 360 for controll names

    public static double getLeftYOP() {
        return operator.getLeftY();
    }

    public static double getLeftTriggerOP() {
        return operator.getLeftTriggerAxis();
    }

    public static double getRightTriggerOP() {
        return operator.getRightTriggerAxis();
    }

    public static boolean getLeftBumperPressedOP() {
        return operator.getLeftBumperPressed();
    }

    public static boolean getRightBumperPressedOP() {
        return operator.getRightBumperPressed();
    }

    public static boolean GetDpadUpOP() {
        if (operator.getPOV() == 0.0) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean GetDpadDownOP() {
        if(operator.getPOV() == 180) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean GetDpadLeftOP() {
        if(operator.getPOV() == 270) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean GetDpadRightOP() {
        if(operator.getPOV() == 90) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean GetBackOP() {
        return operator.getBackButton();
    }

    public static boolean GetYbuttonOP() {
        return operator.getYButton();
    }

    public static boolean GetXbuttonOP() {
        return operator.getXButton();
    }

    public static boolean GetBbuttonOP() {
        return operator.getBButton();
    }

    public static boolean GetAbuttonOP() {
        return operator.getAButton();
    }

    public static boolean GetStartOP() {
        return operator.getStartButton();
    }

    public static double getRightYOP() {
        return operator.getRightY();
    }



}
