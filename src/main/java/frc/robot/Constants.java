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
    public static final int operatorPort = 1;
    public static final double driveDeadband = 0.1;

    public static final double defaultThrottle = 0.8;
    public static final double defaultRotate = 0.8;
    public static final double slowThrottle = 1;
    public static final double slowRotate = 0.8;

    // public static final double poleCatSpeed = 1;
    public static final double poleCatMaxDist = 5;
    public static final double poleCatMinDist = 0;
  }

  public static class EncoderConstants {
    public static final double poleCat = (1./10.) * 2 * Math.PI;
    public static final double encoder = (1./10.75) * (6 * 2.54 * (Math.PI)) * (1./42.);
  }

  public static class IdentifierConstants {
    public static final int driveFL = 7;
    public static final int driveFR = 6;
    public static final int driveBL = 1;
    public static final int driveBR = 10;

    // Catapult 
    public static final int solenoidForward = 1;
    public static final int solenoidReverse = 0;

    // Polecat 
    public static final int polecat = 5;
    public static final int polecatUpperSwitch = 1;
    public static final int polecatLowerSwitch = 2;
  }

  public static class DriveConstants {
    public static final double trackWidth = 0;
  }
}
