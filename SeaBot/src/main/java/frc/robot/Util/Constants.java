// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class Constants {
    public static class DT_PIDF {
        public static final double DT_Left_PG = 0.1;
        public static final double DT_Left_IG = 0.0;
        public static final double DT_Left_DG = 0.0;
        public static final double DT_Left_F = 1023.0/20300.0;

        //public static final double DT_LEFT_MM_Accel = 5000.0;
        //public static final double DT_LEFT_MM_Cruise = 5000.0;

        public static final double DT_Right_PG = 0.1;
        public static final double DT_Right_IG = 0.0;
        public static final double DT_Right_DG = 0.0;
        public static final double DT_Right_F = 1023.0/20300.0;

        //public static final double DT_Right_MM_Accel = 5000.0;
        //public static final double DT_RIGHT_MM_Cruise = 5000.0;

        public static final int DT_SlotIdx = 0;

        public static final int DT_PIDLoopIdx = 0;

        public static final int DT_TimeoutMs = 30;

    }
}
