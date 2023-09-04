// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class ArmSparkMax implements ArmIO {
    private static CANSparkMax UpperArm;
    private static CANSparkMax LowerArm;
    private static CANSparkMax LeftIntake;
    private static CANSparkMax RightIntake;
    private static CANCoder UpperCoder;
    private static CANCoder LowerCoder;
    private final Solenoid Wrist;
    private final Solenoid ClawMode;
    private final Solenoid ArmBrakeU;
    private final Solenoid ArmBrakeL;

    public ArmSparkMax() {
        UpperArm = new CANSparkMax(5, MotorType.kBrushless);
        LowerArm = new CANSparkMax(6, MotorType.kBrushless);

        UpperArm.setInverted(false);
        LowerArm.setInverted(false);

        UpperCoder = new CANCoder(5);
        LowerCoder = new CANCoder(6);

        UpperArm.burnFlash();
        LowerArm.burnFlash();

        ArmBrakeL = new Solenoid(PneumaticsModuleType.REVPH, 1);
        ArmBrakeU = new Solenoid(PneumaticsModuleType.REVPH, 2);
        Wrist = new Solenoid(PneumaticsModuleType.REVPH, 3);
        ClawMode = new Solenoid(PneumaticsModuleType.REVPH, 4);

        LeftIntake = new CANSparkMax(7, MotorType.kBrushless);
        RightIntake = new CANSparkMax(8, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.LowerCoderPosition = LowerCoder.getAbsolutePosition();
        inputs.UpperCoderPosition = UpperCoder.getAbsolutePosition();
        inputs.BrakeL = ArmBrakeL.get();
        inputs.BrakeU = ArmBrakeU.get();
    }


    @Override
    public void setSpeed(double upperSpeed, double lowerSpeed, boolean Intake, boolean Lbrake, boolean Ubrake, boolean IntakeMode, double IntakeSpeed) {
        ArmBrakeL.set(Lbrake);
        ArmBrakeU.set(Ubrake);
        Wrist.set(Intake);
        ClawMode.set(IntakeMode);
        LeftIntake.set(IntakeSpeed);
        RightIntake.set(-IntakeSpeed);

        UpperArm.set(upperSpeed);
        LowerArm.set(lowerSpeed);
    }

}
