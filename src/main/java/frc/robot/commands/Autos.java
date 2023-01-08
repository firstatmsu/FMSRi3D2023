// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drive;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public static CommandBase autonTest(Drive drive) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(0, 0, 0),
            drive.kinematics,
            10
        );


    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(1, 0.5)
            .setKinematics(drive.kinematics)
            .addConstraint(autoVoltageConstraint);


    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);


    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drive::getPose,
            new RamseteController(2, 0.7),
            new SimpleMotorFeedforward(0, 0, 0),
            drive.kinematics,
            drive::getWheelSpeeds,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            // RamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive
            );

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drive.zero());
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
