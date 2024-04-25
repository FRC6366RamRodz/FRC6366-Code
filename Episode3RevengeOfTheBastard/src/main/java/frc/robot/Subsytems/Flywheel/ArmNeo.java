// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

/** Add your docs here. */
public class ArmNeo implements ArmIO {
  private final CANSparkMax UpperArm = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax LowerArm = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax IntakeLeft = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax IntakeRight = new CANSparkMax(8, MotorType.kBrushless); 
  private final SparkLimitSwitch intakeSwitchLeft = IntakeLeft.getForwardLimitSwitch(Type.kNormallyOpen);
  private final SparkPIDController LowerArmPID = LowerArm.getPIDController();
  private final RelativeEncoder UpperNeoEncoder;

 
  private final CANcoder UpperEncoder = new CANcoder(0);
  private final CANcoder LowerEncoder = new CANcoder(1);

  private final int LowerGearRatio = 150;
  private final int UpperGearRatio = 235;
  
  public ArmNeo() {
    UpperNeoEncoder = UpperArm.getEncoder(SparkRelativeEncoder.Type.kQuadrature, (4096*UpperGearRatio));
    

  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.VelocityRPM = 0.0;
    inputs.accelerationRPM = 0.0; // rev doesnt support for some reason

    inputs.Amps = 0.0;
    inputs.Volts = 0.0;

    inputs.Temp = 0.0;
  }

  @Override
  public void setPosition(double UpperAngle, double LowerArm, boolean WristSet, double Intake) {

  }
}
