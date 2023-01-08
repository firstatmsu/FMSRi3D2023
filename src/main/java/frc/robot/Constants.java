// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int controllerPort = 0;

    public static final double throttleMultiplier = 0.1;
    public static final double rotateMultiplier = 0.1;

    public static final double poleCatSpeed = 0.2;
    public static final double poleCatMaxDist = 5;
    public static final double poleCatMinDist = 0;
  }

  public static class IdentifierConstants {
    public static final int driveFL = 0;
    public static final int driveFR = 0;
    public static final int driveBL = 0;
    public static final int driveBR = 0;

    // Catapult 
    public static final int solenoidForward = 0;
    public static final int solenoidReverse = 1;

    // Polecat 
    public static final int polecat = 0;
    public static final int polecatUpperSwitch = 0;
    public static final int polecatLowerSwitch = 0;
  }

  public static class DriveConstants {
    public static final double trackWidth = 0;

    // If gear ratio is wrong, use 8.45
    public static final double encoder = 4096 * (1./10.75) * (6 * 2.54 * (2*Math.PI));
  }
}
