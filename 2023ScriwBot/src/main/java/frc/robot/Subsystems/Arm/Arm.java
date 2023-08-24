// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

/** Add your docs here. */
public class Arm {
    private final ArmIO io;
    private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    public void armPeriodic() {
        io.updateInputs(inputs);
        org.littletonrobotics.junction.Logger.getInstance().processInputs("Arm", inputs);
    }
}
