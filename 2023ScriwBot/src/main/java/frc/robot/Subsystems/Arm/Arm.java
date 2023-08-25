// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class Arm {
    private final ArmIO io;
    private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();
    private Mechanism2d arm = new Mechanism2d(20, 50);
    private MechanismRoot2d root = arm.getRoot("arm", 2, 0);
    private MechanismLigament2d m_upperArm = root.append(new MechanismLigament2d("Upperarm", 10, 45));
    private MechanismLigament2d m_lowerArm = m_upperArm.append(new MechanismLigament2d("lowerArm", 3, 45, 2, new Color8Bit(Color.kYellow)));
    
    public Arm(ArmIO io) {
        this.io = io;
    }

    public void armPeriodic() {
        io.updateInputs(inputs);
        org.littletonrobotics.junction.Logger.getInstance().processInputs("Arm", inputs);
        SmartDashboard.putData("Arm2d", arm);
       // m_lowerArm.setAngle(inputs.LowerCoderPosition);
       // m_upperArm.setAngle(inputs.UpperCoderPosition);
    }
}
