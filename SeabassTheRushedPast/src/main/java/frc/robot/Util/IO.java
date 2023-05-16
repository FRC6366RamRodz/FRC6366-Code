// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Util.Constants.DT_STG;

/** Add your docs here. */
public class IO {
    private static final XboxController driver = new XboxController(0);
    private static final XboxController operator = new XboxController(1);

    public static double joyDeadband(double stickValue) {
        if (Math.abs(stickValue) <= DT_STG.Deadband) {
            return 0;
        } else {
            return stickValue;
        }
    }

    public static double getPOVButton() {
        return driver.getPOV();
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

    public static boolean getXButton() {
        return driver.getXButton();
    }

     //operator
     public static double getLeftYaxisOp() {
        return operator.getLeftY();
    }

    public static boolean getLeftYUpButtonOp() {
        if (operator.getLeftY()>0.5) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean getLeftYDownButtopOp() {
        if (operator.getLeftY()<-0.5) {
            return true;
        } else {
            return false;
        }
    }

    public static double getRightYaxisOp() {
        return operator.getRightY();
    }

    public static boolean getRightYUpButtonOp() {
        if (operator.getRightY()>0.5) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean getRightYDownButtopOp() {
        if (operator.getRightY()<-0.5) {
            return true;
        } else {
            return false;
        }
    } 

    public static double getPovOp() {
        return operator.getPOV();
    }

    public static double getLeftTriggerOp() {
        return operator.getLeftTriggerAxis();
    }

    public static boolean getLeftTriggerSTG1Op() {
       if (operator.getLeftTriggerAxis()>0.1 && operator.getLeftTriggerAxis()<0.90) {
        return true;
       } else {
        return false;
       }
    }

    public static boolean getLeftTriggerSTG2Op() {
        if (operator.getLeftTriggerAxis()>0.85) {
            return true;
        } else {
            return false;
        }
    }

    public static double getRightTriggerOp() {
        return operator.getRightTriggerAxis();
    }

    public static boolean getRightTriggerButtonOp() {
        if (operator.getRightTriggerAxis()>0.4) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean getAButtonPressedOp() {
        return operator.getAButtonPressed();
    }

    public static boolean getBbuttonPressedOp() {
        return operator.getBButtonPressed();
    }

    public static boolean getXButtonPressedOp() {
        return operator.getXButtonPressed();
    }

    public static boolean getYbuttonPressedOp() {
        return operator.getYButtonPressed();
    }

    public static boolean getLeftBumperPressedOp() {
        return operator.getLeftBumperPressed();
    }

    public static boolean getRightBumperPressedOp() {
        return operator.getRightBumperPressed();
    }

    public static boolean getStartButtonPressedOp() {
        return operator.getStartButtonPressed();
    }

    public static boolean getStopButtonPressedOp() {
        return operator.getBackButtonPressed();
    }

    public static boolean getAButtonOp() {
        return operator.getAButton();
    }

    public static boolean getBbuttonOp() {
        return operator.getBButton();
    }

    public static boolean getXButtonOp() {
        return operator.getXButton();
    }

    public static boolean getYbuttonOp() {
        return operator.getYButton();
    }

    public static boolean getLeftBumperOp() {
        return operator.getLeftBumper();
    }

    public static boolean getRightBumperOp() {
        return operator.getRightBumper();
    }

    public static boolean startButtonOp() {
        return operator.getStartButton();
    }

    public static boolean stopButtonOp() {
        return operator.getBackButton();
    }

}
