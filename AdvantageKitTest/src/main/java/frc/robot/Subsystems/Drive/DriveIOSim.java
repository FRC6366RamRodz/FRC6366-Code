// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

/** Add your docs here. */
public class DriveIOSim implements DriveIo{
    private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleFalcon500PerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

    @Override
    public void updateInputs(DriveIOInputs inputs){
        double WHEEL_RADIOUS_METERS = Units.inchesToMeters(3.0);
        sim.update(0.02);

        inputs.leftPositionRad = sim.getLeftPositionMeters() / WHEEL_RADIOUS_METERS;
        inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / WHEEL_RADIOUS_METERS;

        inputs.rightPostitionRad = sim.getRightPositionMeters() / WHEEL_RADIOUS_METERS;
        inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / WHEEL_RADIOUS_METERS;
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        sim.setInputs(MathUtil.clamp(leftVolts, -12.0, 12.0), MathUtil.clamp(rightVolts, -12.0, 12.0));
    }
}
