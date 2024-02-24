// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Arm {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  public XboxController drive = new XboxController(0);
  public static Rotation2d position;

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("angle", inputs);
  }

  public void run() {

    if (drive.getAButton()) {
      position = new Rotation2d(Units.degreesToRadians(90));
    } else {
      position = new Rotation2d(Units.degreesToRadians(0));
    }

    io.setMotors(position);
  }
}
