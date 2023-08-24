// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    public static class ArmIoInputs {
        public double UpperCoderPosition = 0.0;
        public double LowerCoderPosition = 0.0;
        public double UpperMotorVelocity = 0.0;
        public double LowerMotorVelocity = 0.0;
    }

    public default void updateInputs(ArmIoInputs inputs) {
    }

    public default void setSpeed(double upperSpeed, double lowerSpeed, double UsetPoint, double LsetPoint) {
    }
}
