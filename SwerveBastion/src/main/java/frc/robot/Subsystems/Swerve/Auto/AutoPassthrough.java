// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm.Arm;

/** Add your docs here. */
public class AutoPassthrough extends CommandBase {
    double ArmMode;

    public class IntakeDown extends CommandBase {
        public IntakeDown(){
        }
        @Override
        public void execute() {
            ArmMode = 1;
        }
    }


    public double commandOutput() {
        return ArmMode;
    }
    
}
