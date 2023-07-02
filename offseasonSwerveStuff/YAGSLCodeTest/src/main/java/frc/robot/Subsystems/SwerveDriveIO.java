// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public interface SwerveDriveIO {
    public static class SwerveDriveIOInputs{

    }

    public default void updateInputs(SwerveDriveIOInputs inputs) {
    }

    public default void runSwerve(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    }
}
