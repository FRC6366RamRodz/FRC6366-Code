// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class IntakeREV implements IntakeIO {
  public static final Solenoid intake = new Solenoid(PneumaticsModuleType.REVPH, 1);
  public static final CANSparkMax intakeMotor = new CANSparkMax(6, MotorType.kBrushless);
  public boolean intakeSet;

  public IntakeREV() {
    intakeMotor.restoreFactoryDefaults(false);
    intakeMotor.setCANTimeout(250);
    intakeMotor.setInverted(false);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setSmartCurrentLimit(20);
    intakeMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeSpeed =
        intakeMotor.getEncoder().getVelocity(); // may need to be disabled based on motor in use;
    inputs.extended = intakeSet;
  }

  @Override
  public void setIntake(boolean extended, double motorVolts) {
    intake.set(extended);
    intakeSet = extended;
    intakeMotor.set(motorVolts);
  }
}
