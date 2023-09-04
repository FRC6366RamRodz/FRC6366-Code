// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Util.RobotContants;

/** Add your docs here. */
public class ArmSim implements ArmIO {
    private SingleJointedArmSim upperArm = new SingleJointedArmSim(DCMotor.getNEO(1), 45.0, 0.08, RobotContants.UpperArmLengthinMeter , Units.degreesToRadians(-90), Units.degreesToRadians(90), false);
    private SingleJointedArmSim lowArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), 25.0, 0.004, RobotContants.LowerArmLengthinMeter, Units.degreesToRadians(-120), Units.degreesToRadians(120), false);
    private SingleJointedArmSim endaffector = new SingleJointedArmSim(DCMotor.getNeo550(1), 1, 0.008, RobotContants.IntakelengthinMeter, Units.degreesToRadians(10), Units.degreesToRadians(90), false);
    private boolean BrakeU = false;
    private boolean BrakeL = false;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        upperArm.update(0.02);
        lowArmSim.update(0.02);
        endaffector.update(0.02);

        inputs.UpperCoderPosition = Units.radiansToDegrees(lowArmSim.getAngleRads());
        inputs.LowerCoderPosition = Units.radiansToDegrees(lowArmSim.getAngleRads());
        inputs.BrakeL = BrakeL;
        inputs.BrakeU = BrakeU;

    }

    @Override
    public void setSpeed(double upperSpeed, double lowerSpeed, boolean Intake, boolean Lbrake, boolean Ubrake, boolean IntakeMode, double IntakeSpeed) {
        double upperSpeed2, lowerSpeed2, intakeSpeed;

        if (BrakeL == true) {
            lowerSpeed2 = 0;
        } else {
            lowerSpeed2 = lowerSpeed;
        }

        if (BrakeU == true) {
            upperSpeed2 = 0;
        } else {
            upperSpeed2 = lowerSpeed;
        }

        BrakeL = Lbrake;
        BrakeU = Ubrake;

        upperArm.setInput(MathUtil.clamp(upperSpeed2*12, -12.0, 12.0));
        lowArmSim.setInput(MathUtil.clamp(lowerSpeed2*12, -12.0, 12.0));

        if (Intake) {
            intakeSpeed = 12;
        } else {
            intakeSpeed = -12;
        }

        endaffector.setInput(intakeSpeed);
    }

}
