// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ControllsProcessing;

/** Add your docs here. */
public class ClawControlls {
    public static double clawMotor(boolean trigger1, boolean trigger2, boolean clawMode) {
            double clawMotor;
        if (clawMode == true) {
            if (trigger2 == true) {
                clawMotor = -0.2;
              } else if (trigger1 == true) {
                clawMotor = 0.35;
              } else {
                clawMotor = 0;
              }
        } else {
            if (trigger2 == true) {
                clawMotor = -0.2;
              } else if (trigger1 == true) {
                clawMotor = 0.35;
              } else {
                clawMotor = 0;
              }
        }
        return clawMotor;
    }
}
