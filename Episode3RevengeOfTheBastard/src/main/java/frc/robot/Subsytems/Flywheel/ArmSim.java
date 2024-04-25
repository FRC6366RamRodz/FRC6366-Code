// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

/** Add your docs here. */
public class ArmSim implements ArmIO {


  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.VelocityRPM = 0.0;
    inputs.accelerationRPM = 0.0;

    inputs.Amps = 0.0;
    inputs.Volts = 0.0;

    inputs.Temp = 0.0;
  }

  @Override
  public void setPosition(double UpperAngle, double LowerArm, boolean WristSet, double Intake) {
  }
}
