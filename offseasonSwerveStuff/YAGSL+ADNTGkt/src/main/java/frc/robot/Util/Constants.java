// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import swervelib.parser.PIDFConfig;

/** Add your docs here. */
public class Constants {
    public static class Swrv_STG {
        public static final double DED_BND = 0.15;
    }

    public static class Auton {
        
        public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);
    
        public static final double MAX_SPEED        = 4;
        public static final double MAX_ACCELERATION = 2;
    }
}
