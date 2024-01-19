// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;


/** Add your docs here. */
public class Shooter {  
    private Pose3d angle = new Pose3d();
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void ShooterPeriodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Angle", angle);
  }

  public void teleop() {
    io.setMotors(0, 0, 0, 0, 0);
    angle = new Pose3d(0, 0, 0, new Rotation3d(0,0,0));
  }

  public double getAnlge() {
    return inputs.anglePosition;
  }
}
