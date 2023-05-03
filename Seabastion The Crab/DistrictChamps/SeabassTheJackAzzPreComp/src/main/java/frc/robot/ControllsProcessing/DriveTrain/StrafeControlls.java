// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ControllsProcessing.DriveTrain;

import frc.robot.Util.Constants.DT_Set;

/** Add your docs here. */
public class StrafeControlls {
    
    public static double StrafeS(double forward, double rotate) {
        double fward = Math.abs(forward);
        double sens = DT_Set.DT_TURN_SENSITIVITY;
            if (forward == 0) {
                fward = 1;
                sens = DT_Set.DT_QUICK_TURN;
            }
        double left = -forward + rotate * fward * sens;
        return left;
    }

    public static double StingerS(double forward, double rotate) {
        double fward = Math.abs(forward);
        double sens = DT_Set.DT_TURN_SENSITIVITY;
            if (forward == 0) {
                fward = 1;
                sens = DT_Set.DT_QUICK_TURN;
            }
        double right = -forward - rotate * fward * sens;
        return right;
    }

}
