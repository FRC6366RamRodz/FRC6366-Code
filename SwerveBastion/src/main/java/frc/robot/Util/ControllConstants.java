// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import swervelib.parser.PIDFConfig;

/** Add your docs here. */
public class ControllConstants {
    public static final double DeadBand = 0.14;
    public static final double DeadBandTriger = 0.05;

    public static final Pose2d panicPose = new Pose2d(1.84, 0.48, new Rotation2d(0));

    public static final double trimLeftOffset = 0.05;
    public static final double trimRightOffset = -0.05;

    public static final double peakSlowMode = 0.5;

    public static class Auton {
        
        public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);
    
        public static final double MAX_SPEED        = 4;
        public static final double MAX_ACCELERATION = 2;
    }
}
