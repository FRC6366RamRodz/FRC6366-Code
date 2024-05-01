// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch;

/** Add your docs here. */
public class ArmNeo implements ArmIO {
  //N = Neo1.1, f = 550, V = Vortex.
  private final CANSparkMax N_UpperArm = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax N_LowerArm = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax f_IntakeLeft = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax f_IntakeRight = new CANSparkMax(8, MotorType.kBrushless); 
  private final SparkLimitSwitch intakeSwitchLeft = f_IntakeLeft.getForwardLimitSwitch(Type.kNormallyOpen);
  private final Solenoid UpperBrake = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private final Solenoid LowerBrake = new Solenoid(PneumaticsModuleType.REVPH, 1);
  private final Solenoid Wrist = new Solenoid(PneumaticsModuleType.REVPH, 2);

  private final ArmFeedforward lowerArmFF = new ArmFeedforward(0, 0, 0);
  private final PIDController lowerArmPID = new PIDController(0, 0, 0);
  private final PIDController upperArmPid = new PIDController(0, 0, 0);
  private final ArmFeedforward upperArmFF = new ArmFeedforward(0, 0, 0);

 
  private final CANcoder UpperEncoder = new CANcoder(0);
  private final CANcoder LowerEncoder = new CANcoder(1);
  private final StatusSignal<Double> LowerAbsolutePosition;
  private final StatusSignal<Double> UpperAbsolutePosition;
  
  public ArmNeo() {

    lowerArmPID.enableContinuousInput(-180/360, 180/360);
    upperArmPid.enableContinuousInput(-180/360, 180/360);

    N_UpperArm.clearFaults();
    N_UpperArm.restoreFactoryDefaults();
    N_UpperArm.clearFaults();
    N_UpperArm.enableVoltageCompensation(12);
    N_UpperArm.setInverted(false);
    N_UpperArm.setIdleMode(IdleMode.kCoast);
    N_UpperArm.burnFlash();
    N_UpperArm.clearFaults();

    N_LowerArm.clearFaults();
    N_LowerArm.restoreFactoryDefaults();
    N_LowerArm.clearFaults();
    N_LowerArm.enableVoltageCompensation(12);
    N_LowerArm.setInverted(false);
    N_LowerArm.setIdleMode(IdleMode.kCoast);
    N_LowerArm.burnFlash();
    N_LowerArm.clearFaults();

    f_IntakeLeft.clearFaults();
    f_IntakeLeft.restoreFactoryDefaults();
    f_IntakeLeft.clearFaults();
    f_IntakeLeft.enableVoltageCompensation(12);
    f_IntakeLeft.setInverted(false);
    f_IntakeLeft.setIdleMode(IdleMode.kCoast);
    f_IntakeLeft.setSmartCurrentLimit(30);
    f_IntakeLeft.burnFlash();
    f_IntakeLeft.clearFaults();
    
    f_IntakeRight.clearFaults();
    f_IntakeRight.restoreFactoryDefaults();
    f_IntakeRight.clearFaults();
    f_IntakeRight.enableVoltageCompensation(12);
    f_IntakeRight.setInverted(false);
    f_IntakeRight.setIdleMode(IdleMode.kCoast);
    f_IntakeRight.setSmartCurrentLimit(30);
    f_IntakeRight.burnFlash();
    f_IntakeRight.clearFaults();

    intakeSwitchLeft.enableLimitSwitch(true);

    LowerAbsolutePosition = LowerEncoder.getAbsolutePosition();
    UpperAbsolutePosition = UpperEncoder.getAbsolutePosition();

    LowerAbsolutePosition.setUpdateFrequency(350);
    UpperAbsolutePosition.setUpdateFrequency(350);

    LowerEncoder.optimizeBusUtilization();
    UpperEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    BaseStatusSignal.refreshAll(LowerAbsolutePosition, UpperAbsolutePosition);

    inputs.UpperArmPosition = UpperAbsolutePosition.getValueAsDouble();
    inputs.LowerArmPosition = LowerAbsolutePosition.getValueAsDouble();
    inputs.UpperTemp = N_UpperArm.getMotorTemperature();
    inputs.LowerTemp = N_LowerArm.getMotorTemperature();
    inputs.UpperPIDError = upperArmPid.getPositionError();
    inputs.LowerPIDError = lowerArmPID.getPositionError();
    inputs.IntakeLeftSpeed = f_IntakeLeft.getAppliedOutput()*917;
    inputs.IntakeRightSpeed = f_IntakeRight.getAppliedOutput()*917;
    inputs.UpperBrake = UpperBrake.get();
    inputs.LowerBrake = LowerBrake.get();
    inputs.IntakeWrist = Wrist.get();
  }

  @Override
  public void setPosition(double UpperAngle, double LowerArm, boolean WristSet, double Intake) {

    N_LowerArm.setVoltage(lowerArmPID.calculate(LowerEncoder.getPosition().getValueAsDouble(), LowerArm)+lowerArmFF.calculate(LowerArm, 0));
    
    N_UpperArm.setVoltage(upperArmPid.calculate(UpperEncoder.getPosition().getValueAsDouble(), UpperAngle)+upperArmFF.calculate(UpperAngle, 0));

    f_IntakeLeft.setVoltage(Intake);
    f_IntakeRight.setVoltage(Intake);

    Wrist.set(WristSet);

    boolean upperBrake, lowerBrake;

    if (lowerArmPID.getPositionError() > 0.5) {
      lowerBrake = true;
    } else {
      lowerBrake = false;
    }

    if (upperArmPid.getPositionError() > 0.5) {
      upperBrake = true;
    } else {
      upperBrake = false;
    }

    UpperBrake.set(upperBrake);
    LowerBrake.set(lowerBrake);
    
  }
}
