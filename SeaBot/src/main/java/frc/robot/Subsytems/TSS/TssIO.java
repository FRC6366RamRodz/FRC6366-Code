// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.TSS;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TssIO {
    @AutoLog
    public static class TssIOInputs {
        
    }

    

    public default void UpdateInputs(TssIOInputs inputs) {
    }

    public default void drive(double Left, double Right, double stingFront, double stingRear, boolean Strafe, boolean Sringer) {
    }

    public default void autonomous(double left, double right) {
    }
}
