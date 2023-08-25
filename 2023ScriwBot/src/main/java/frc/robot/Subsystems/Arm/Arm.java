// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Util.Constants;
import frc.robot.Util.IO;

/** Add your docs here. */
public class Arm {
    private final ArmIO io;
    private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();

    private Mechanism2d arm = new Mechanism2d(20, 50);
    private MechanismRoot2d root = arm.getRoot("arm", 0, 14);
    private MechanismLigament2d m_upperArm = root.append(new MechanismLigament2d("Upperarm", 10, -90));
    private MechanismLigament2d m_lowerArm = m_upperArm.append(new MechanismLigament2d("lowerArm", 3, 120, 2, new Color8Bit(Color.kYellow)));

    private PIDController UarmPID;
    private PIDController LarmPID;
    
    public Arm(ArmIO io) {
        this.io = io;

        UarmPID = new PIDController(Constants.AR_SET.kP, Constants.AR_SET.kI, Constants.AR_SET.kD);
        LarmPID = new PIDController(Constants.AR_SET.kP, Constants.AR_SET.kI, Constants.AR_SET.kD);

    }

    public void armPeriodic() {
        io.updateInputs(inputs);
        org.littletonrobotics.junction.Logger.getInstance().processInputs("Arm", inputs);
        SmartDashboard.putData("Arm2d", arm);
    }

    public void setPointMode() {
       double Uspeed, Uspeed2, Lspeed, Lspeed2, UsetPoint, LsetPoint;

       if (IO.getOpA()) {
        LsetPoint = 0;
        UsetPoint = -80;
       } else if (IO.getOpB()) {
        LsetPoint = 0;
        UsetPoint = 0;
       } else if (IO.getOpX()) {
        LsetPoint = 0;
        UsetPoint = -10;
       } else if (IO.getOpY()) {
        LsetPoint = 0;
        UsetPoint = -40;
       } else {
        LsetPoint = 120;
        UsetPoint = -90;
       }

       Uspeed = UarmPID.calculate(getUpperCoder(), UsetPoint);
       if (Math.abs(Uspeed)>1) {
        Uspeed2 = 1;
       } else {
        Uspeed2 = Uspeed;
       }

       Lspeed = LarmPID.calculate(getLowerCoder(), LsetPoint);
       if (Math.abs(Lspeed)>1) {
        Lspeed2 = 1;
       } else {
        Lspeed2 = Lspeed;
       }

        io.setSpeed(Uspeed2, Lspeed2, UsetPoint, LsetPoint);
        m_lowerArm.setAngle(getLowerCoder());
        m_upperArm.setAngle(getUpperCoder());
    }

    public double getLowerCoder() {
        return inputs.LowerCoderPosition;
    }

    public double getUpperCoder() {
        return inputs.UpperCoderPosition;
    }
}
