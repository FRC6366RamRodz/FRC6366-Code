// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void ArmPeriodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Flywheel", inputs);
  }
}
