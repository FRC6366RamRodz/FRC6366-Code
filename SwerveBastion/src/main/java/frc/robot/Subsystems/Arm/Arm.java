// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Util.ControllConstants;
import frc.robot.Util.IO;

/** Add your docs here. */
public class Arm {
    private Mechanism2d arm = new Mechanism2d(3, 2);
    private MechanismRoot2d root = arm.getRoot("arm", 1.6, Units.inchesToMeters(46));
    private MechanismLigament2d m_upperArm = root.append(new MechanismLigament2d("Upperarm", Units.inchesToMeters(38), -90, 9 , new Color8Bit(Color.kSilver)));
    private MechanismLigament2d m_lowerArm = m_upperArm.append(new MechanismLigament2d("lowerArm", Units.inchesToMeters(15), 120, 8, new Color8Bit(Color.kSilver)));
    private MechanismLigament2d m_intake = m_lowerArm.append(new MechanismLigament2d("intake", Units.inchesToMeters(10), 0, 2 , new Color8Bit(Color.kBlack)));
    private boolean HighPos, MidPos, snglSttn, dbleSttn, clawMode;
    private double offset=0;
    private Timer upDebounce;
    private Timer downDebounce;


    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private PIDController UarmPID;
    private PIDController LarmPID;

    public Arm(ArmIO io) {
        this.io = io; 

        UarmPID = new PIDController(ControllConstants.ARM.kP, ControllConstants.ARM.kI, ControllConstants.ARM.kD);
        LarmPID = new PIDController(ControllConstants.ARM.kP, ControllConstants.ARM.kI, ControllConstants.ARM.kD);

        HighPos = false;
        MidPos = false;
        clawMode = false;
    }

    public void armPeriodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);

        Logger.getInstance().recordOutput("Arm2d", arm);     
    }

    public void SetPointMode() {
        double UsetPoint, LsetPoint, setArm, setElbow, ArmVolt, elbowVolt, ArmVolt2, elbowVolt2, setArm2=0;
        boolean wristFlag;


//Arm
        if (IO.getLeftBumperPressedOP()) {
            HighPos = !HighPos;
        } else if (IO.getRightBumperPressedOP()) {
            HighPos = false;
        }

        if (IO.getRightBumperPressedOP()) {
            MidPos = !MidPos;
        } else if (IO.getLeftBumperPressedOP()) {
            MidPos = false;
        }

        if (IO.getRightTriggerOP()>0.1 && IO.getRightTriggerOP() <0.95) {
            snglSttn = true;
            dbleSttn = false;
        } else if (IO.getRightTriggerOP() > 0.95) {
            snglSttn = false;
            dbleSttn = true;
        } else {
            snglSttn = false;
            dbleSttn = false;
        }

        
        if (IO.getLeftTrigger()) {
            UsetPoint = -74;
            LsetPoint = -8;
        } else if (HighPos) {
            UsetPoint = -7.5;
            LsetPoint = 8;
        } else if (MidPos) {
            UsetPoint = -43;
            LsetPoint = 46;
        } else if (snglSttn) {
            UsetPoint = -80;
            LsetPoint = 30;
        } else if (dbleSttn) {
            UsetPoint = 0;
            LsetPoint = 0;
        } else {
            UsetPoint = -90;
            LsetPoint = 71;
        }

        if (IO.GetBackOP()){
            setArm = getUpperCoder();
        } else {
            setArm = UsetPoint;
        }

        if (getUpperCoder() >= setArm2-0.15 && getUpperCoder() <= setArm2+0.15) {

            if (getUpperCoder() < -20) {
                setElbow = 90-Math.abs(getUpperCoder())+19;
                wristFlag = false;
            } else {
                setElbow = LsetPoint;   
                wristFlag = true;
            }

        } else if(IO.GetBackOP()) {
            setElbow = getLowerCoder();
            wristFlag = false;
        } else {
            setElbow = LsetPoint;
            wristFlag = true;
        }

        if (IO.GetDpadUpOP() == true) {
            upDebounce.start();
        } else {
            upDebounce.stop();
            upDebounce.reset();
        }

        if (IO.GetDpadDownOP() == true) {
            downDebounce.start();
        } else {
            downDebounce.stop();
            downDebounce.reset();
        }

        if (IO.GetDpadUpOP() && upDebounce.get() < 0.1) {
            offset = offset + 2;
        } else if (IO.GetDpadDownOP() && downDebounce.get() < 0.1){
            offset = offset -2;
        } else if (IO.GetStartOP()) {
            offset = 0;
        }


        double score;
        if (IO.getLeftBumper()) {
            score = 5;
        } else {
            score = 0;
        }

        setArm2 = setArm+offset-score;
        
        ArmVolt = UarmPID.calculate(getUpperCoder(), setArm2+offset-score);
        if (Math.abs(ArmVolt)>1) {
          ArmVolt2 = Math.abs(ArmVolt)/ArmVolt;
        } else {
            ArmVolt2 = ArmVolt;
        }

        elbowVolt = LarmPID.calculate(getLowerCoder(), setElbow);
        if (Math.abs(elbowVolt)>1) {
            elbowVolt2 = Math.abs(ArmVolt)/ArmVolt;
        } else {
            elbowVolt2 = elbowVolt;
        }

        
//Brakes
        boolean Ubrake, Lbrake;
        if (getUpperCoder() >= setArm2-0.15 && getUpperCoder() <= setArm2+0.15) {
            Ubrake = false;
        } else {
            Ubrake = true;
        }

        if (getLowerCoder() >= setElbow-0.15 && getLowerCoder() <= setElbow+0.15) {
            Lbrake = false;
        } else {
            Lbrake = true;
        }


//Claw
        if (IO.getRightYOP()>0) {
            clawMode = !clawMode;
        }

        boolean wrist, wrist2;
        if (IO.getLeftY()>0) {
            wrist = true;
        } else if (wristFlag){
            wrist = true;
        } else {
            wrist = false;
        }

        boolean intakeIn, IntakeOut;
        if (IO.getLeftTriggerOP()>0.1 && IO.getLeftTriggerOP()<0.95) {
            intakeIn = true;
            IntakeOut = false;
        } else if (IO.getLeftTriggerOP()>0.95) {
            intakeIn = false;
            IntakeOut = true;
        } else {
            intakeIn = false;
            IntakeOut = false;  
        }

        boolean claw2;
        double IntakeSet;
        if (IntakeOut || IO.getLeftBumper()) {
            if (wrist && IO.getLeftBumper()) {
                claw2 = false;
                IntakeSet = 0.0;
            } else {
                claw2 = clawMode;
                IntakeSet = 0.2;
            }
        } else if (intakeIn || IO.getLeftTrigger()) {
            IntakeSet = -0.2;
            claw2=wrist;
        } else {
            IntakeSet = 0;
            claw2=wrist;
        }

        io.setSpeed(ArmVolt2, elbowVolt2, wrist, Lbrake, Ubrake, claw2, IntakeSet);

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
