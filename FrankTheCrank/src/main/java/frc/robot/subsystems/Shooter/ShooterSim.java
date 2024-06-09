// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ShooterSim implements ShooterIO {
  //arm is based off anderson layout. currently a 135 to 1.
  private SingleJointedArmSim shooterAngle = new SingleJointedArmSim(DCMotor.getFalcon500(1), 200, 0.7480059831487, Units.inchesToMeters(21.729), Units.degreesToRadians(-50), Units.degreesToRadians(90), true, Units.degreesToRadians(-50));
  private FlywheelSim intake = new FlywheelSim(DCMotor.getNEO(1), 5, 0.004);
  private FlywheelSim handler = new FlywheelSim(DCMotor.getNEO(1), 5, 0.04);
  private FlywheelSim feeder = new FlywheelSim(DCMotor.getNeo550(1), 15, 0.04);
  private FlywheelSim topShooter = new FlywheelSim(DCMotor.getKrakenX60(1), 2, 0.004);
  private FlywheelSim bottomShooter = new FlywheelSim(DCMotor.getKrakenX60(1), 2, 0.004);

  private PIDController anglePID = new PIDController(0.2, 0, 0.0000);
  private ArmFeedforward angleFeedForward = new ArmFeedforward(0, 0.0405, 8);

  private PIDController topPID = new PIDController(0.01, 0, 0.0000);
  private PIDController bottomPID = new PIDController(0.01, 0, 0.0000);
  private SimpleMotorFeedforward topFeed = new SimpleMotorFeedforward(0.010, 0.003876);
  private SimpleMotorFeedforward bottomFeed = new SimpleMotorFeedforward(0.010, 0.003876);

  public ShooterSim() {
    anglePID.enableContinuousInput(-180, 180);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shooterAngle.update(0.02); //all sim hardware must be updated periodically
    intake.update(0.02);
    feeder.update(0.02);
    topShooter.update(0.02);
    bottomShooter.update(0.02);
    handler.update(0.02);

    inputs.TopVelocity = topShooter.getAngularVelocityRPM();
    inputs.BottomVelocity = bottomShooter.getAngularVelocityRPM();

    inputs.HandlerVelocity = handler.getAngularVelocityRPM();
    inputs.feederVelocity = feeder.getAngularVelocityRPM();
    inputs.intakeVelocity = intake.getAngularVelocityRPM();

    inputs.anglePosition = Units.radiansToDegrees(shooterAngle.getAngleRads());
    inputs.angleVelocity = Units.radiansPerSecondToRotationsPerMinute(shooterAngle.getVelocityRadPerSec());

    inputs.intakeLimit = true;
  }

  @Override
  public void setMotors(double TopVelocity, double BottomVelocity, double HandlerVelocity, double anglePosition, double intakeVelocity, double feederVelocity, boolean limitOff, double climb) {

    double angleVelocity = angleFeedForward.calculate(anglePosition, Units.degreesToRadians(anglePosition) - shooterAngle.getAngleRads()) + anglePID.calculate(Units.radiansToDegrees(shooterAngle.getAngleRads()), anglePosition);
    shooterAngle.setInputVoltage(angleVelocity);

    double topVelocity = topFeed.calculate(TopVelocity) + topPID.calculate(topShooter.getAngularVelocityRPM(), TopVelocity);
    topShooter.setInputVoltage(topVelocity);

    double bottomVelocity = bottomFeed.calculate(BottomVelocity) + bottomPID.calculate(bottomShooter.getAngularVelocityRPM(), BottomVelocity);
    bottomShooter.setInputVoltage(bottomVelocity);

    intake.setInputVoltage(intakeVelocity * 12);

    feeder.setInput(feederVelocity);

    handler.setInput(HandlerVelocity);
  }
}
