// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmSim implements ArmIO {
  public final int LowerGearRatio = 150;
  public final int UpperGearRatio = 235;

  private SingleJointedArmSim upperArm = new SingleJointedArmSim(DCMotor.getNEO(1), UpperGearRatio, 0, 0, 0, 0, false, 0);

  public ArmSim() {
    
  }


  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.UpperArmPosition = 0.0;
    inputs.LowerArmPosition = 0.0;
    inputs.UpperTemp = 0.0;
    inputs.LowerTemp = 0.0;
    inputs.UpperPIDError = 0.0;
    inputs.LowerPIDError = 0.0;
    inputs.IntakeLeftSpeed = 0.0;
    inputs.IntakeRightSpeed = 0.0;
    inputs.UpperBrake = true;
    inputs.LowerBrake = true;
    inputs.IntakeWrist = false;
  }

  @Override
  public void setPosition(double UpperAngle, double LowerArm, boolean WristSet, double Intake) {
  }
}
