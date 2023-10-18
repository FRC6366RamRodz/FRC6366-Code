// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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
    private Timer LBrake;
    private Timer debounce;
    private Timer debounceee;
    private Timer UBrakeT;


    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private PIDController UarmPID;
    private PIDController LarmPID;

    public Arm(ArmIO io) {
        this.io = io; 

        UarmPID = new PIDController(ControllConstants.ARM.kP, ControllConstants.ARM.kI, ControllConstants.ARM.kD, 0.02);
        LarmPID = new PIDController(ControllConstants.ARM.kP, ControllConstants.ARM.kI, ControllConstants.ARM.kD, 0.02);

        upDebounce = new Timer();
        downDebounce = new Timer();
        LBrake = new Timer();
        debounce = new Timer();
        debounceee = new Timer();
        UBrakeT = new Timer();
    }

    public void armPeriodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);

        Logger.getInstance().recordOutput("Arm2d", arm);     
    }

    public void AutoMode(double Uarm, double Larm, double IntakeSpeed, boolean intakeMode, boolean wrist) {

        double UpArm = MathUtil.clamp(UarmPID.calculate(getUpperCoder() ,Uarm), -1, 1);
        double LoArm = MathUtil.clamp(LarmPID.calculate(getLowerCoder(), Larm), -1, 1);

        boolean Ubrake, Lbrake;
        if (getUpperCoder() >= Uarm-0.15 && getUpperCoder() <= Uarm+0.15) {
            Ubrake = false;
        } else {
            Ubrake = true;
        }

        if (getLowerCoder() >= Larm-0.15 && getLowerCoder() <= Larm+0.15) {
            Lbrake = false;
        } else {
            Lbrake = true;
        }

        io.setSpeed(UpArm, LoArm, wrist, Lbrake, Ubrake, intakeMode, IntakeSpeed);
        m_lowerArm.setAngle(getLowerCoder()-getUpperCoder());
        m_upperArm.setAngle(getUpperCoder());
        m_intake.setAngle(getIntake());
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
        if (IO.getLeftTrigger()){ //floor intake
            UsetPoint = 99;
            LsetPoint = 122;
            wristTst = true;
        } else if (HighPos) {
            UsetPoint = 173;
            LsetPoint = 156;
            wristTst = true;
        } else if (MidPos) {
            UsetPoint = 135;
            LsetPoint = 87;
            wristTst = true;
        } else if (snglSttn) {
            UsetPoint = 130;
            LsetPoint = 50;
            wristTst = true;
        } else if (dbleSttn) {
            UsetPoint = 180;
            LsetPoint = 180;
            wristTst = true;
        } else {
            UsetPoint = 88;
            LsetPoint = 67;
            wristTst = false;
        }

        if (IO.GetBackOP()){
            setArm = UsetPoint; //getUpperCoder();
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

        setArm2 = setArm;//+offset-score;

        if (getUpperCoder()<160) {

            if ((getUpperCoder() >= setArm2-10 && getUpperCoder() <= setArm2+10) == true) {
                setElbow = LsetPoint;
            } else {
                setElbow = 90;   
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
        
        UarmPID.setSetpoint(setArm2);
        UarmPID.setTolerance(0.1);
        ArmVolt = UarmPID.calculate(getUpperCoder(), setArm2);
        ArmVolt2 = MathUtil.clamp(ArmVolt, -1, 1);

        LarmPID.setSetpoint(setElbow);
        LarmPID.setTolerance(0.1);
        elbowVolt = LarmPID.calculate(getLowerCoder());
        elbowVolt2 = MathUtil.clamp(elbowVolt, -1, 1);

        
//Brakes
        boolean Ubrake, Lbrake;
        if (getUpperCoder() >= setArm2-3 && getUpperCoder() <= setArm2+3) {
            UBrakeT.start();
            if (UBrakeT.get() > 1) {
                Ubrake = false;
            } else {
                Ubrake = true;
            }

        } else {
            UBrakeT.stop();
            UBrakeT.reset();
            Ubrake = true;
        }

        if (getLowerCoder() >= setElbow-3 && getLowerCoder() <= setElbow+3) {
            LBrake.start();
            if (LBrake.get() > 1) {
                Lbrake = false;
            } else {
                Lbrake = true;
            }
        } else {
            LBrake.stop();
            LBrake.reset();
            Lbrake = true;
        }


//wristFlag
        if (getUpperCoder() >= setArm2-5 && getUpperCoder() <= setArm2+5) {
            wristFlag = true;
        } else {
            wristFlag = false;
        }
        



//Claw

boolean ccont;
        if (IO.getRightYOP()>0.5) {
            debounceee.start();
            if(debounceee.get()<0.01) {
                ccont = true;
            } else {
                ccont = false;
            }
        } else {
            debounceee.stop();
            debounceee.reset();
            ccont = false;
        }

        if (ccont) {
            clawMode = !clawMode;
        }


        boolean wcont;
        if (IO.getLeftYOP()>0.5) {
            debounce.start();
            if(debounce.get()<0.02) {
                wcont = true;
            } else {
                wcont = false;
            }
        } else {
            debounce.stop();
            debounce.reset();
            wcont = false;
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
