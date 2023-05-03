// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class Constants {

    public static class DT_Map {
        public static final int leftFront = 0;
        public static final int righFront = 1;
        public static final int leftRear = 0;
        public static final int rightRear = 1;
        public static final int frontH = 2;
        public static final int rearH = 3;
        public static final int stingerSolenoid = 0;
        public static final int strafeSolenoid = 1;
    }

    public static class DT_Set{
        public static final double DT_DEADBAND = 0.1;
        public static final double DT_TURN_SENSITIVITY = 0.6;
        public static final double DT_quickTurn = 1;
        public static final double DT_slowSpeed = 0.5;
    }
}
