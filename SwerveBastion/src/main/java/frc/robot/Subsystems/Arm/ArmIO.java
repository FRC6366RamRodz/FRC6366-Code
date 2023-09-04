// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
            public double UpperCoderPosition = 0.0;
            public double LowerCoderPosition = 0.0;
            public boolean BrakeU = false;
            public boolean BrakeL = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setSpeed(double upperSpeed, double lowerSpeed, boolean Intake, boolean Lbrake, boolean Ubrake, boolean IntakeMode, double IntakeSpeed) {
    }
}
