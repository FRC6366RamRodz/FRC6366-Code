// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public interface SwerveIO {
    @AutoLog
    public static class SwerveIoInputs {

    }

    public default void updateInputs(SwerveIoInputs inputs) {
    }

    public default void run(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    }
}
