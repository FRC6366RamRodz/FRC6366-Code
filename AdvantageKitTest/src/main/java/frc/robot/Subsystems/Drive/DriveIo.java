// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface DriveIo {
    @AutoLog
    public static class DriveIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double rightPostitionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(DriveIOInputs inputs) {
    }

    public default void setVoltage(double leftVolts, double rightVolts) {
    }
}
