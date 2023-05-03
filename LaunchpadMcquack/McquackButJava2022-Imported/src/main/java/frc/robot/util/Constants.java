// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class Constants { 

    public static class Map{
        public static final int LEFT_BACK_DRIVETRAIN = 3;

        public static final int RIGHT_BACK_DRIVETRAIN = 4;

        public static final int LEFT_FRONT_DRIVETRAIN = 1;

        public static final int RIGHT_FRONT_DRIVETRAIN = 2;

        public static final int FRONT_INTAKE_MOTOR = 4;
        public static final int BACK_INTAKE_MOTOR = 3;
        public static final int FRONT_INTAKE_SOLENOID = 0;
        public static final int BACK_INTAKE_SOLENOID = 1; 

        public static final int WINCH_MOTOR = 2;
        public static final int ARM_ANGLE_MOTOR = 7;
    }

    public static class SH_Set{
        /**
			 * Convert 500 RPM to units / 100ms.
			 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
        public static final double SH_VelocitySet = 500.0 * 4096 / 600;

        public static final int SH_ShooterTop = 4;
        public static final int SH_ShooterBottom = 5;
        public static final int SH_Tower = 7; 
    }

    public static class SH_PIDF {
        public static final double SH_Left_PG = 0.1;
        public static final double SH_Left_IG = 0.0;
        public static final double SH_Left_DG = 0.0;
        public static final double SH_Left_F = 1023.0/20300.0;

        //public static final double DT_LEFT_MM_Accel = 5000.0;
        //public static final double DT_LEFT_MM_Cruise = 5000.0;

        public static final double SH_Right_PG = 0.1;
        public static final double SH_Right_IG = 0.0;
        public static final double SH_Right_DG = 0.0;
        public static final double SH_Right_F = 1023.0/20300.0;

        public static final int SH_SlotIdx = 0;

        public static final int SH_PIDLoopIdx = 0;

        public static final int SH_TimeoutMs = 30;

    }

    public static class DT_Set{
        public static final double DT_DEADBAND = 0.1; 
        public static final double DT_TURN_SENSITIVITY = 0.65;
        public static final double DT_QUICK_TURN = 0.6;
        public static final double DT_MAX_VELOCITY = 18000.0;
        public static final double DT_RAMP_RATE_SECS = 0.5;

        public static final double AUTO_DISTANCE_LEFT_1_1 = 176000;
        public static final double AUTO_DISTANCE_RIGHT_1_1 = 176000;
        public static final double AUTO_DISTANCE_LEFT_1_2 = 88000;
        public static final double AUTO_DISTANCE_RIGHT_1_2 = 88000;

        public static final double MM_ACCELERATION = 6000.0;
        public static final double MM_CRUISECONTROL = 6000.0;

        public static final double INTAKE_SPEED = 0.5;

    }

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
