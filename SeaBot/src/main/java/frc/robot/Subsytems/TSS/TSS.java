// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.TSS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Util.IO;

/** Add your docs here. */
public class TSS {
    private final TssIO io;
    private final TssIOInputsAutoLogged inputs = new TssIOInputsAutoLogged();


    public TSS(TssIO io) {
        this.io = io;
    }

    public void periodicDrive() {
        io.UpdateInputs(inputs);
        org.littletonrobotics.junction.Logger.getInstance().processInputs("Drive", inputs);
    }

    public void Tank( double Lefty, double Rightx) {

        double xSpeed = Math.pow(MathUtil.applyDeadband(Lefty, 0.2), 3); 
        double zRotation = Math.pow(MathUtil.applyDeadband(Rightx, 0.2), 3); 

        var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, false);

        io.drive(speeds.left*12, speeds.right*12, 0, 0, false, false);
    }

    public void Strafe(double Leftx, double Rightx) {

        double xSpeed = Math.pow(MathUtil.applyDeadband(Leftx, 0.2), 3); 
        double zRotation = Math.pow(MathUtil.applyDeadband(Rightx, 0.2), 3);

        
        var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, false);

        io.drive(0, 0, speeds.left, speeds.right, true, true);
    }

    public void Stigner(double Leftx, double Lefty, double Rightx, double Lefttrig, double Righttrig) {

        double LeftY = Math.pow(MathUtil.applyDeadband(Lefty, 0.2), 3);
        double LeftX = Math.pow(MathUtil.applyDeadband(Leftx, 0.2), 3);
        double RightX = Math.pow(MathUtil.applyDeadband(Rightx, 0.2), 3);
        double LeftTrig = Math.pow(MathUtil.applyDeadband(Lefttrig, 0.2), 3);
        double RightTrig = Math.pow(MathUtil.applyDeadband(Righttrig, 0.2), 3);

        double counter = 0.2;
        double LEFT = (LeftY - (LeftX * counter)) + RightX;
        double Right = (LeftY + (LeftX * counter)) + RightX;
        double Stinger = (LeftX + (-(LeftTrig) + RightTrig));

        io.drive(LEFT, Right, 0, Stinger, false, true);
    }

    public void stop() {
        io.drive(0, 0, 0, 0, false, false);
    }

    public void TSSLogic(boolean sting, boolean strafe, boolean tank, boolean slow) {

        double slowVal = 0.5;

        double LeftY = IO.getLeftY();
        double LeftX = IO.getLeftX();
        double RightX = IO.getRightX();
        double LeftTrig = IO.getLeftTrigger();
        double RightTrig = IO.getLeftTrigger();

        double leftY;
        double leftX;
        double rightX;
        double leftTrig;
        double rightTrig;

        if (slow) {
            leftY = LeftY * slowVal;
            leftX = LeftX * slowVal;
            rightX = RightX * slowVal;
            leftTrig = LeftTrig * slowVal;
            rightTrig = RightTrig * slowVal;
        } else {
            leftY = LeftY;
            leftX = LeftX;
            rightX = RightX;
            leftTrig = LeftTrig;
            rightTrig = RightTrig;
        }

        if (sting) {
            Stigner(leftX, leftY, rightX, leftTrig, rightTrig);
        } else if (strafe) {
            Strafe(leftX, rightX);
        } else if (tank) {
            Tank(leftY, rightX);
        } else {
            Tank(leftY, rightX);
        }
    }

    public void auto() {
        io.autonomous(-10, -10);
    }


}
