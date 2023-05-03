// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Util.IO;

/** Add your docs here. */
public class SubClaw {
    private SparkMaxLimitSwitch L_CLAW_FORWARD_LIMIT;
    private SparkMaxLimitSwitch L_CLAW_REVERSE_LIMIT;
    private SparkMaxLimitSwitch R_CLAW_FORWARD_LIMIT;
    private SparkMaxLimitSwitch R_CLAW_REVERSE_LIMIT;
    private CANSparkMax L_CLAW_MOTOR;
    private CANSparkMax R_CLAW_MOTOR;
    private final Solenoid WRIST;
    private final Solenoid CLAW;
    private final Solenoid CONE;
    private final Solenoid CUBE;
    private final Solenoid CLAW_MODE;

    public SubClaw(int lClaw, int rClaw, int claw, int wrist, int cube, int cone, int mode) {
        //initialize motors
        L_CLAW_MOTOR = new CANSparkMax(lClaw, MotorType.kBrushless);
        R_CLAW_MOTOR = new CANSparkMax(rClaw, MotorType.kBrushless);
        //reset Motors
        L_CLAW_MOTOR.restoreFactoryDefaults();
        R_CLAW_MOTOR.restoreFactoryDefaults();
        //initialize pnuematics
        WRIST = new Solenoid(PneumaticsModuleType.REVPH, wrist);
        CLAW = new Solenoid(PneumaticsModuleType.REVPH, claw);
        CONE = new Solenoid(PneumaticsModuleType.REVPH, cone);
        CUBE = new Solenoid(PneumaticsModuleType.REVPH, cube);
        CLAW_MODE = new Solenoid(PneumaticsModuleType.REVPH, mode);
        //initialize limit switch
        L_CLAW_FORWARD_LIMIT = L_CLAW_MOTOR.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        L_CLAW_REVERSE_LIMIT = L_CLAW_MOTOR.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        R_CLAW_FORWARD_LIMIT = R_CLAW_MOTOR.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        R_CLAW_REVERSE_LIMIT = R_CLAW_MOTOR.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        //enable limit switches
        L_CLAW_FORWARD_LIMIT.enableLimitSwitch(true);
        L_CLAW_REVERSE_LIMIT.enableLimitSwitch(true);
        R_CLAW_FORWARD_LIMIT.enableLimitSwitch(true);
        R_CLAW_REVERSE_LIMIT.enableLimitSwitch(true);

        R_CLAW_MOTOR.setSmartCurrentLimit(30);
        L_CLAW_MOTOR.setSmartCurrentLimit(30);

        L_CLAW_MOTOR.burnFlash();
        R_CLAW_MOTOR.burnFlash();
    }

    public void runClaw(double lClaw, double rClaw, boolean wrist, boolean cone, boolean cube, boolean clawMode) {
        double Lclaw, Rclaw, Lclaw2, Rclaw2;
        boolean Wrist;

        if (IO.getAbutton() == true) {
            Wrist = true;
            Lclaw = 0.4;
            Rclaw = -0.4;
        } else {
        Wrist = wrist;
        Lclaw = lClaw;
        Rclaw = rClaw;
        }
        if (clawMode == true) {
            Lclaw2 = Lclaw*2;
            Rclaw2 = Rclaw*2;
        } else {
            Lclaw2 = Lclaw;
            Rclaw2 = Rclaw;
        }
        //set motors
        L_CLAW_MOTOR.set(Lclaw2);
        R_CLAW_MOTOR.set(Rclaw2);
        //set wrist
        WRIST.set(Wrist);
        //set claw
        CLAW.set(clawMode);
        //set lights
        CONE.set(cone);
        CUBE.set(cube);
        CLAW_MODE.set(clawMode);

    }
}
