// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class Constants {

    public static class DT_STG/*Drive Train Settings */ {//global varriables shoule be cammel case and abbreviations with underscores seperatring words
        public static final double Dt_Tnk_Acc_Rte = 0.3;//drive train tank acceleration rate
        public static final double Dt_Str_Acc_Rte = 0.1;//drive train strafe acceleration rate
        public static final int Pk_Crrnt_Lmt = 80; //peak current limit
        public static final int Pk_Crrnt_Lmt_Tm = 250; //peak current limit time in mil sec (1000 = 1 sec)(how long it can go beyond current limit)
        public static final double Slw_Md_Tnk_Spd = 0.3;//(slow mode tank speed)max speed percentage for tank mode in slow mode
        public static final double Slw_Md_Stig_Spd = 0.5;//(slow mode stinger speed)max speed percentage for stinger mode in slow mode
        public static final double Slw_Md_Str_Spd = 0.4;
        public static final double Deadband = 0.05;
        public static final double Drv_Sens = 0.55;
    }

     //Arm Settings
     public static class AR_SET {
        public static final double kP = 6e-5; 
        public static final double kI = 0;
        public static final double kD = 0; 
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

    public static class DT_Map{
        public static final int leftFront = 0;
        public static final int rightFront = 1;
        public static final int leftRear = 0;
        public static final int rightRear = 1;
        public static final int stinger = 2;
        public static final int strafe = 3;
        public static final int stingerSolenoid = 5;
        public static final int strafeSolenoid = 6;
    }
            //arm
    public static class AR_Map {
        public static final int armMotor = 7;
        public static final int elbowMotor = 8;
        public static final int armBrake = 1;
        public static final int elbowBrake = 0;
    }

        //claw
        public static class CW_Map{
            public static final int lClawMotor = 9;
            public static final int rClawMotor = 10; 
            public static final int clawPnuematic = 2;
            public static final int wristPnumatic = 3;
            public static final int clawModePnuematic = 4;
            public static final int coneLight = 9;
            public static final int cubeLight = 8;
        }
}
