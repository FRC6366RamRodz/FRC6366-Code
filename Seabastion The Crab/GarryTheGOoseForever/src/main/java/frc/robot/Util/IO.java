// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Util.Constants.DT_Set;

/** Add your docs here. */
public class IO {

    private static final XboxController driver = new XboxController(0);
    private static final XboxController operator = new XboxController(1);
  

    
/*  Toggle looks like this
        if(driver.getLeftBumperPressed()) {
            solenoid.set(!solenoid.get())
         }
*/
public static double joyLimiter(double input) {
    SlewRateLimiter filter = new SlewRateLimiter(Constants.DT_Set.DT_ACCrateS);
    return filter.calculate(input);
}

//deadband
    public static double joyDeadband(double stickValue) {
        if (Math.abs(stickValue) <= DT_Set.DT_DEADBAND) {
            return 0;
        } else {
         return stickValue;
        }
        

    }
                

    //driver controlls
    public static double getLeftY() {
        return joyDeadband(driver.getLeftY());
    }

    public static double getLeftX() {
        return joyDeadband(driver.getLeftX());
    }

    public static double getRightX(){
        return joyDeadband(driver.getRightX());
    }

    //steering mode for triggers
    public static double getTriggerSteer() {
        if (driver.getLeftTriggerAxis() >= driver.getRightTriggerAxis()) {
            return driver.getLeftTriggerAxis();
        } else {
            return -driver.getRightTriggerAxis();
        }
    }

    //get Bumpers
    public static boolean getLeftBumper() {
        return driver.getLeftBumper();
    }

    public static boolean getRightBumper() {
        return driver.getRightBumper();
    }

    // get bumper pressed (only give one signal when presed versus constant)
    public static boolean getLeftBumperPressed() {
        return driver.getLeftBumperPressed();
    }

    public static boolean getRightBumperPressed() {
        return driver.getRightBumperPressed();
    }

    //get ABXY
    public static boolean getAbutton() {
        return driver.getAButton();
    }

    public static boolean getBbutton() {
        return driver.getYButton();
    }

    public static boolean getXbutton() {
        return driver.getXButton();
    }

    public static boolean getYbutton() {
        return driver.getYButton();
    }

    //get ABXY pressed (only give one signal when presed versus constant)
    public static boolean getAbuttonPressed() {
        return driver.getAButtonPressed();
    }

    public static boolean getBbuttonPressed() {
        return driver.getBButtonPressed();
    }

    public static boolean getXbuttonPressed() {
        return driver.getXButtonPressed();
    }

    public static boolean getYbuttonPressed() {
        return driver.getYButtonPressed();
    }

    //get POV (D-pad) values 

    public static double getPOVButton() {
        return driver.getPOV();
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
