// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

    private Mechanism2d arm = new Mechanism2d(3, 2);
    private MechanismRoot2d root = arm.getRoot("arm", 1.6, Units.inchesToMeters(46));
    private MechanismLigament2d m_upperArm = root.append(new MechanismLigament2d("Upperarm", Units.inchesToMeters(38), -90, 9 , new Color8Bit(Color.kSilver)));
    private MechanismLigament2d m_lowerArm = m_upperArm.append(new MechanismLigament2d("lowerArm", Units.inchesToMeters(15), 120, 8, new Color8Bit(Color.kSilver)));
    private MechanismLigament2d m_intake = m_lowerArm.append(new MechanismLigament2d("intake", Units.inchesToMeters(10), 0, 2 , new Color8Bit(Color.kBlack)));

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
        Logger.getInstance().recordOutput("Arm2d", arm);

        double Lx = Units.inchesToMeters(38)*Math.cos(Units.degreesToRadians(getUpperCoder()+360));
        double La = (Units.degreesToRadians(getUpperCoder()+90))-Units.degreesToRadians(180);
        double Lh = Lx/Math.cos(La);
        Logger.getInstance().recordOutput("lowerArm", new Pose3d((Lh*(Math.cos(La))), 0, Lx, new Rotation3d(0, Units.degreesToRadians(0), 0)));
        //Logger.getInstance().recordOutput("lowerArm", new Pose3d(new Translation3d(Lh, new Rotation3d(0, Units.degreesToRadians(getUpperCoder()), 0)), new Rotation3d(0, Units.degreesToRadians(0), 0)));

    }

    public void setPointMode() {
       double Uspeed, Uspeed2, Lspeed, Lspeed2, UsetPoint, LsetPoint;

       if (IO.getOpA()) {
        LsetPoint = -8;
        UsetPoint = -74;
       } else if (IO.getOpB()) {
        LsetPoint = 8;
        UsetPoint = -7.5;
       } else if (IO.getOpX()) {
        LsetPoint = 46;
        UsetPoint = -43;
       } else if (IO.getOpY()) {
        LsetPoint = 0;
        UsetPoint = 0;
       } else {
        LsetPoint = 25;
        UsetPoint = -90;
       }

       Uspeed = UarmPID.calculate(getUpperCoder(), UsetPoint);
       if (Math.abs(Uspeed)>1) {
        Uspeed2 = Uspeed/Uspeed;
       } else {
        Uspeed2 = Uspeed;
       }

       Lspeed = LarmPID.calculate(getLowerCoder(), LsetPoint);
       if (Math.abs(Lspeed)>1) {
        Lspeed2 = Lspeed/Lspeed;
       } else {
        Lspeed2 = Lspeed;
       }

        io.setSpeed(Uspeed2, Lspeed2, UsetPoint, LsetPoint);
        m_lowerArm.setAngle(getLowerCoder()-getUpperCoder());
        m_upperArm.setAngle(getUpperCoder());
    }

    public double getLowerCoder() {
        return inputs.LowerCoderPosition;
    }

    public double getUpperCoder() {
        return inputs.UpperCoderPosition;
    }
}
