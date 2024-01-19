// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Intake {
    private final IntakeIO io;
    //private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void IntakePeriodic() {
       // io.updateInputs(inputs);
      //  Logger.processInputs("Intake", inputs);
    }

    public void runIntake() {
        io.setIntake(false, 0);
    }
}
