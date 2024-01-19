// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Flywheel {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void FlywheelPeriodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Flywheel", inputs);
  }

  public void velocitySpinUp() {
    io.setVelocity(0);
  }

  public void velocityHold() {
    io.setVelocity(0);
  }

  public void stop() {
    io.setVelocity(0);
  }
}
