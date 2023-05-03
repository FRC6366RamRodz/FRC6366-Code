// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class Constants {
    //Drive Train settings
    public static class DT_Set{
        public static final double DT_ACCrate = 1;
        public static final double DT_ACCrateS = 1;
        public static final double DT_DEADBAND = 0.10; 
        public static final double DT_TURN_SENSITIVITY = 0.5;
        public static final double DT_QUICK_TURN = 0.5;
        public static final double kP = 0.00005; 
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0.000015; 
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
        public static final double maxRPM = 20000;
        public static final double MaxStrRPM = 20000;
        public static final double maxVel = 12000; // rpm
        public static final double maxAcc = 8000;
        public static final double minVel = 0;
        public static final double allowedErr = 0;
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

    //motor/solenoid maps
    public static class DT_Map{
        public static final int leftFront = 1;
        public static final int rightFront = 2;
        public static final int leftRear = 3;
        public static final int rightRear = 4;
        public static final int stinger = 5;
        public static final int strafe = 6;
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
        public static final int coneLight = 7;
        public static final int cubeLight = 8;
    }
}
