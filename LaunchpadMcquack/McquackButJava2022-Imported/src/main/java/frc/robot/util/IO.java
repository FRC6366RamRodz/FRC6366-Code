// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;


import edu.wpi.first.wpilibj.XboxController;
/** Add your docs here. */
public class IO {

    private static final XboxController driver = new XboxController(0);
    
    
    //Driver Controlls
    public static double getLeftY() {
        return Math.pow(joyDeadband(driver.getLeftY()), 2) * Math.signum(driver.getLeftY());
    }

    public static double getRightX() {
        return Math.pow(joyDeadband(driver.getRightX()), 2) * Math.signum(driver.getRightX());
    }

    public static double joyDeadband(double stickValue) {
        if (stickValue <= Constants.DT_Set.DT_DEADBAND && stickValue >= -Constants.DT_Set.DT_DEADBAND) {
            return 0;
        } else {
         return stickValue;
        }
        

    }
 
    public static double joySquareValue(double rawValue) {
        return Math.signum(rawValue) *rawValue*rawValue;
    }

    public static boolean getLeftBumper(){
        return driver.getLeftBumper();
    }

    public static boolean getRightTriggerAxis(){
        if(driver.getRightTriggerAxis() >= 0.1){
            return true;
        } else {
            return false;
        }
    }

    public static boolean getLeftTriggerAxis(){
        if(driver.getLeftTriggerAxis() >= 0.1){
            return true;
        } else {
            return false;
        }
    }

    public static boolean getXbutton() {
        return driver.getXButton();
    }

    public static boolean getBbutton() {
        return driver.getBButton();
    }

    public static boolean getAbutton() {
        return driver.getAButton();
    }

    public static boolean getYbutton() {
        return driver.getYButton();
    }

    public static boolean getXbuttonPressed() {
        return driver.getXButtonPressed();
    }

    public static boolean getXbuttonRealeased() {
        return driver.getXButtonReleased();
    }

    public static void testing(String[] args) {
        System.out.println(getRightX());  
    }

}
