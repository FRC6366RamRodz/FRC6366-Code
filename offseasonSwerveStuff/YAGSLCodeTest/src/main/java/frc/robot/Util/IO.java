// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;


import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {

    private static final XboxController driver = new XboxController(0);

    public static double setDeadzone(double input) {
        double Input = Math.abs(input);

        if (Input < 0.10) {
            return 0;
        } else {
            return input;
        }
    }
    public static double getLeftY() {
        return setDeadzone(driver.getLeftY());
    }

    public static double getLeftX(){
        return setDeadzone(driver.getLeftX());
    }

    public static double getRightX() {
        return setDeadzone(driver.getRightX());
    }




}
