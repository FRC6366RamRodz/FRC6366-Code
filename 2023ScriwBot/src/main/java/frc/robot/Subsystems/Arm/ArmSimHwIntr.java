// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ArmSimHwIntr implements ArmIO {
    private Mechanism2d arm = new Mechanism2d(0, 0);
    private MechanismRoot2d root = arm.getRoot("arm", 2, 0);
    private MechanismLigament2d m_upperArm = root.append(new MechanismLigament2d("Upperarm", 4, 45));
    private MechanismLigament2d m_lowerArm = m_upperArm.append(new MechanismLigament2d("lowerArm", 3, 0, 1, new Color8Bit(Color.kYellow)));

    @Override
    public void updateInputs(ArmIoInputs inputs) {
        inputs.LowerCoderPosition = m_lowerArm.getAngle();
        inputs.UpperCoderPosition = m_upperArm.getAngle();
        SmartDashboard.putData("Arm2d", arm);
    }

    @Override
    public void setSpeed(double upperSpeed, double lowerSpeed, double UsetPoint, double LsetPoint) {
        m_upperArm.setAngle(UsetPoint);
        m_lowerArm.setAngle(LsetPoint);
    }
}
