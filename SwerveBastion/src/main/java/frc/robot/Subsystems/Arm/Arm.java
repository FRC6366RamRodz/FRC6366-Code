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
    private boolean HighPos=false, snglSttn, dbleSttn, clawMode=false;
    private boolean MidPos=false;
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

        upDebounce = new Timer();
        downDebounce = new Timer();
    }

    public void armPeriodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);

        Logger.getInstance().recordOutput("Arm2d", arm);     
    }

    public void SetPointMode() {
        double UsetPoint, LsetPoint, setArm, setElbow, ArmVolt, elbowVolt, ArmVolt2, elbowVolt2, setArm2;
        boolean wristFlag;


//Arm
        if (IO.getRightBumperPressedOP()) {
            MidPos = !MidPos;
            HighPos = false;
        } else if (IO.getLeftBumperPressedOP()) {
            MidPos = false;
            HighPos = !HighPos;
        }

        if (IO.getRightTriggerOP()>0.15 && IO.getRightTriggerOP() <0.95) {
            snglSttn = true;
            dbleSttn = false;
        } else if (IO.getRightTriggerOP() > 0.95) {
            snglSttn = false;
            dbleSttn = true;
        } else {
            snglSttn = false;
            dbleSttn = false;
        }

        boolean wristTst;
        if (IO.getLeftTrigger()) {
            UsetPoint = -74;
            LsetPoint = -8;
            wristTst = true;
        } else if (HighPos) {
            UsetPoint = -7.5;
            LsetPoint = 8;
            wristTst = true;
        } else if (MidPos) {
            UsetPoint = 0;
            LsetPoint = 0;
            wristTst = true;
        } else if (snglSttn) {
            UsetPoint = -80;
            LsetPoint = 30;
            wristTst = true;
        } else if (dbleSttn) {
            UsetPoint = 0;
            LsetPoint = 0;
            wristTst = true;
        } else {
            UsetPoint = -90;
            LsetPoint = 19;
            wristTst = false;
        }

        if (IO.GetBackOP()){
            setArm = getUpperCoder();
        } else {
            setArm = UsetPoint;
        }

        if (IO.GetDpadUpOP() && upDebounce.get() < 0.02) {
            offset = offset + 1;
        } else if (IO.GetDpadDownOP() && downDebounce.get() < 0.02){
            offset = offset -1;
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

        if ((getUpperCoder() >= setArm2-10 && getUpperCoder() <= setArm2+10) == false) {

            if (getUpperCoder() < -20) {
                setElbow = 90-Math.abs(getUpperCoder())+19;
            } else {
                setElbow = LsetPoint;   
            }

        } else if(IO.GetBackOP()) {
            setElbow = getLowerCoder();
        } else {
            setElbow = LsetPoint;
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
        
        ArmVolt = UarmPID.calculate(getUpperCoder(), setArm2);
        if (Math.abs(ArmVolt)>1) {
          ArmVolt2 = Math.abs(ArmVolt)/ArmVolt;
        } else {
            ArmVolt2 = ArmVolt;
        }

        elbowVolt = LarmPID.calculate(getLowerCoder(), setElbow);
        if (Math.abs(elbowVolt)>1) {
            elbowVolt2 = Math.abs(elbowVolt)/elbowVolt;
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

        if (getUpperCoder() >= setArm2-5 && getUpperCoder() <= setArm2+5) {
            wristFlag = true;
        } else {
            wristFlag = false;
        }
        


//Claw
        if (IO.getRightYOP()>0) {
            clawMode = !clawMode;
        }

        boolean wrist;
        if ((wristFlag && wristTst) || IO.getLeftYOP()>0.5) {
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
            claw2=clawMode;
        } else {
            IntakeSet = 0;
            claw2=clawMode;
        }

        io.setSpeed(ArmVolt2, elbowVolt2, wrist, Lbrake, Ubrake, claw2, IntakeSet);

        m_lowerArm.setAngle(getLowerCoder()-getUpperCoder());
        m_upperArm.setAngle(getUpperCoder());
        m_intake.setAngle(getIntake());

    }

    public double getLowerCoder() {
        return inputs.LowerCoderPosition;
    }

    public double getUpperCoder() {
        return inputs.UpperCoderPosition;
    }

    public double getIntake() {
        return inputs.IntakePosition;
    }
}
