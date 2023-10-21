// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Auto.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Swerve.SwerveIO;
import frc.robot.Subsystems.Swerve.SwerveSparkMax;

public class AutoBalance extends CommandBase {
  private SwerveSparkMax swerve;
  private PIDController Balan;
  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveSparkMax swerve) {
    this.swerve = swerve;
    Balan = new PIDController(0.87, 0, 0);
    Balan.setTolerance(6);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output;
    output = Balan.calculate(swerve.getPitch(),0);
    RobotContainer.Swerve.absoluteDrive(0, -output, 0, -1, false, false, false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
