// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

/** Add your docs here. */
public class DriveSim implements DriveIO {
  private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k12p75, KitbotWheelSize.kSixInch, null);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    sim.update(0.02);
    inputs.leftPositionMeter = sim.getLeftPositionMeters();
    inputs.leftVelocity = sim.getLeftVelocityMetersPerSecond();
    inputs.leftAvgVolts = leftAppliedVolts;
    inputs.leftAvgAmps = sim.getLeftCurrentDrawAmps();

    inputs.rightPositionMeter = sim.getRightPositionMeters();
    inputs.rightVelocity = sim.getRightVelocityMetersPerSecond();
    inputs.rightAvgAmps = rightAppliedVolts;
    inputs.rightAvgVolts = sim.getRightCurrentDrawAmps();

    inputs.gyroYaw = new Rotation2d(Units.rotationsToRadians((inputs.rightPositionMeter - inputs.leftPositionMeter) / (Units.inchesToMeters(28)*Math.PI)));
    //inputs.gyroYaw = sim.getHeading();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftAppliedVolts = MathUtil.clamp(leftVolts * 12, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts * 12, -12.0, 12.0);
    sim.setInputs(leftAppliedVolts, rightAppliedVolts);
  }
}
