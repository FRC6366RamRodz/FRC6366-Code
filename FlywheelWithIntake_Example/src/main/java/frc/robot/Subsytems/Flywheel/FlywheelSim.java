// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class FlywheelSim implements FlywheelIO {
  private final DCMotorSim flywheelSim = new DCMotorSim(DCMotor.getNEO(1), 2 / 1, 0.025);
  private double inputVolts = 0.0;
  private final PIDController PID = new PIDController(0.1, 0, 0, 0.02);

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    inputs.VelocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.accelerationRPM = 0.0; // sim doesnt support for some reason

    inputs.Amps = flywheelSim.getCurrentDrawAmps();
    inputs.Volts = inputVolts;

    inputs.Temp = 0.0;
  }

  @Override
  public void setVelocity(double VelocityRPM) {
    double PIDOut = PID.calculate(flywheelSim.getAngularVelocityRPM(), VelocityRPM);
    flywheelSim.setInputVoltage(PIDOut);
  }
}
