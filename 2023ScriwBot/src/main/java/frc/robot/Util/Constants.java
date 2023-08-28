// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class Constants {

    public static final Mode currentMode = Mode.SIM;
    public static enum Mode {
        /** Running on a real robot. */
        REAL,
    
        /** Running a physics simulator. */
        SIM,
    
        /** Replaying from a log file. */
        REPLAY
      }

    public static class AR_SET {
        public static final double kP = 0.009; 
        public static final double kI = 0;
        public static final double kD = 0.0001; 
        public static final double kIz = 0; 
        public static final double kFF = 0.000015; 
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
        public static final double maxRPM = 20000;
        public static final double maxVel = 20000;
        public static final double maxAcc = 20000;
        public static final double minVel = 0;
        public static final double allowedErr = 0.01;
    }

    public static class DT_STG/*Drive Train Settings */ {
        public static final double DeadBand = 0.05; //sometimes an abbreviation is more problamatic in those cases leave them camel case
        public static final double Drv_Sens = 0.55;//driver sensitivity
    }
}
