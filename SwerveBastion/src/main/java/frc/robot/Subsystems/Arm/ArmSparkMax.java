// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Subsystems.Swerve.Auto.Commands.IntakeDown;
import frc.robot.Util.ControllConstants.ARM;

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
    private SparkMaxLimitSwitch ArmUSwitch;
    private SparkMaxLimitSwitch ArmLSwitch;
    private SparkMaxLimitSwitch intakeSwitchl;
    private SparkMaxLimitSwitch intakeSwithc2;

    public ArmSparkMax() {
        UpperArm = new CANSparkMax(5, MotorType.kBrushless);
        LowerArm = new CANSparkMax(6, MotorType.kBrushless);

        UpperArm.setInverted(false);
        LowerArm.setInverted(false);

        UpperCoder = new CANCoder(5);
        LowerCoder = new CANCoder(6);
        UpperCoder.configMagnetOffset(118.03);
        LowerCoder.configMagnetOffset(-122.87);

        UpperArm.burnFlash();
        LowerArm.burnFlash();

        ArmBrakeL = new Solenoid(PneumaticsModuleType.REVPH, 0);
        ArmBrakeU = new Solenoid(PneumaticsModuleType.REVPH, 1);
        Wrist = new Solenoid(PneumaticsModuleType.REVPH, 3);
        ClawMode = new Solenoid(PneumaticsModuleType.REVPH, 2);

        LeftIntake = new CANSparkMax(7, MotorType.kBrushless);
        RightIntake = new CANSparkMax(8, MotorType.kBrushless);

        ArmUSwitch = UpperArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ArmLSwitch = UpperArm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ArmUSwitch.enableLimitSwitch(true);
        ArmLSwitch.enableLimitSwitch(true);
        intakeSwitchl = LeftIntake.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        intakeSwithc2 = RightIntake.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        intakeSwitchl.enableLimitSwitch(true);
        intakeSwithc2.enableLimitSwitch(true);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.LowerCoderPosition = LowerCoder.getAbsolutePosition();
        inputs.UpperCoderPosition = UpperCoder.getAbsolutePosition();
        inputs.BrakeL = ArmBrakeL.get();
        inputs.BrakeU = ArmBrakeU.get();

        double wristset;
        if(Wrist.get()) {
            wristset = 90;
        } else {
            wristset = 10;
        }
        inputs.IntakePosition = wristset;
    }


    @Override
    public void setSpeed(double upperSpeed, double lowerSpeed, boolean Intake, boolean Lbrake, boolean Ubrake, boolean IntakeMode, double IntakeSpeed) {
        ArmBrakeL.set(Lbrake);
        ArmBrakeU.set(Ubrake);
        Wrist.set(Intake);
        ClawMode.set(IntakeMode);
        LeftIntake.set(-IntakeSpeed);
        RightIntake.set(-IntakeSpeed);

        UpperArm.set(upperSpeed);
        LowerArm.set(lowerSpeed);
    }

    public double getUArm() {
        return UpperArm.get();
    }

    public double getLarm() {
        return LowerArm.get();
    }

}
