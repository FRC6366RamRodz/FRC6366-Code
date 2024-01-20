// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ShooterSim implements ShooterIO {
  private SingleJointedArmSim ShooterAngle =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          50,
          0.008,
          Units.inchesToMeters(1),
          Units.degreesToRadians(-90),
          Units.degreesToRadians(90),
          false,
          0);
  private FlywheelSim intake = new FlywheelSim(DCMotor.getNEO(1), 1, 0.01);
  private FlywheelSim feeder = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.01);
  private FlywheelSim TopShooter = new FlywheelSim(DCMotor.getKrakenX60(1), 1, 0.01);
  private FlywheelSim BottomShooter = new FlywheelSim(DCMotor.getKrakenX60(1), 1, 0.01);

  private PIDController angleSet = new PIDController(0.7, 0, 0);
  private PIDController topShooter = new PIDController(0.02, 0, 0.0);
  private PIDController bottomShooter = new PIDController(0.02, 0, 0);
  private PIDController Feeder = new PIDController(0.08, 0, 0);
  private PIDController Intake = new PIDController(0.02, 0, 0);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    ShooterAngle.update(0.02);
    intake.update(0.02);
    feeder.update(0.02);
    TopShooter.update(0.02);
    BottomShooter.update(0.02);

    inputs.TopVelocity = TopShooter.getAngularVelocityRPM();
    inputs.BottomVelocity = BottomShooter.getAngularVelocityRPM();
    inputs.anglePosition = Units.radiansToDegrees(ShooterAngle.getAngleRads());
    inputs.angleVelocity =
        Units.radiansPerSecondToRotationsPerMinute(ShooterAngle.getVelocityRadPerSec());
    inputs.feederVelocity = feeder.getAngularVelocityRPM();
    inputs.intakeVelocity = intake.getAngularVelocityRPM();
  }

  @Override
  public void setMotors(
      double TopVelocity,
      double BottomVelocity,
      double feederVelocity,
      double anglePosition,
      double intakeVelocity) {
    double AngleSet =
        angleSet.calculate(Units.radiansToDegrees(ShooterAngle.getAngleRads()), anglePosition);
    ShooterAngle.setInputVoltage(MathUtil.clamp(AngleSet, -12.0, 12.0));
 
    double topVelocity = topShooter.calculate(TopShooter.getAngularVelocityRPM(), TopVelocity);
    TopShooter.setInputVoltage(MathUtil.clamp(topVelocity, -12.0, 12.0));

    double bottomVelocity =
        bottomShooter.calculate(BottomShooter.getAngularVelocityRPM(), -BottomVelocity);
    BottomShooter.setInputVoltage(MathUtil.clamp(bottomVelocity, -12.0, 12.0));

    double FeederVelocity =
        Feeder.calculate(feeder.getAngularVelocityRPM(), feeder.getAngularVelocityRPM());
    feeder.setInputVoltage(FeederVelocity);

    double IntakeVelocity = Intake.calculate(intake.getAngularVelocityRPM(), intakeVelocity);
    intake.setInputVoltage(IntakeVelocity);
  }
}
