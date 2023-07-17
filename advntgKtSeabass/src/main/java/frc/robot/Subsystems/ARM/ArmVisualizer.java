// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ARM;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ArmVisualizer {
    private final Mechanism2d ARM;
    private final MechanismRoot2d root;
    private final MechanismLigament2d m_UpperArm;
    private final MechanismLigament2d m_LowerArm;

    public ArmVisualizer() {

    ARM = new Mechanism2d(3, 3, new Color8Bit(Color.kSilver));
    root = ARM.getRoot("ARM", 2, 0);
    m_UpperArm = root.append(new MechanismLigament2d("upperArm", 50, 0, 4, new Color8Bit(Color.kSilver)));
    m_LowerArm = root.append(new MechanismLigament2d("lowerArm", 50, 90, 4, new Color8Bit(Color.kRed)));

    }
}
