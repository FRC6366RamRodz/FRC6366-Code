// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

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

    public static boolean getRightBumper() {//autoDrive
       return driver.getRightBumper();
    }

    public static boolean getYbuttonPressed() {//lockPose
        return driver.getYButtonPressed();
    }


}
