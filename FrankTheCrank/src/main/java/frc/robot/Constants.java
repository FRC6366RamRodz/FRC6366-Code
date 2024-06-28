// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;//Mode switch

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

   public static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
      ;

//Camera Positions.
  public static Transform3d frontLeftCamera = new Transform3d(0.4, 0.363, 0.33, new Rotation3d(Units.degreesToRadians(-32),Units.degreesToRadians(-37), 0));
  public static Transform3d frontRightCamera = new Transform3d(-0.4,0.4,0.33, new Rotation3d(Units.degreesToRadians(-18), Units.degreesToRadians(-37), 0));
  public static Transform3d BackLeftCamera = new Transform3d(-0.4,0.307,0.33, new Rotation3d(Units.degreesToRadians(-270), Units.degreesToRadians(-45), Units.degreesToRadians(180)));
  public static Transform3d BackRightCamera = new Transform3d(-0.7,-0.0207,0.33, new Rotation3d(Units.degreesToRadians(-178), Units.degreesToRadians(-45), Units.degreesToRadians(0))); 
  /*public static Transform3d frontLeftCamera = new Transform3d(0.1778, 0.33, 0.33, new Rotation3d(Units.degreesToRadians(-32),Units.degreesToRadians(-37), 0));
  public static Transform3d frontRightCamera = new Transform3d(-0.1778,0.33,0.33, new Rotation3d(Units.degreesToRadians(-18), Units.degreesToRadians(-37), 0));
  public static Transform3d BackLeftCamera = new Transform3d(0.291,0.33,-0.36, new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(45), Units.degreesToRadians(180-45)));
  public static Transform3d BackRightCamera = new Transform3d(-0.291,-0.33,-0.36, new Rotation3d(0, Units.degreesToRadians(45), Units.degreesToRadians(180+45)));*/
}
