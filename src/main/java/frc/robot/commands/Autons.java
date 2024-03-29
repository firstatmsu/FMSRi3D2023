// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PneumaticCatapult;

/** Add your docs here. */
public class Autons {
    public static CommandBase forwardAuton(Drive drive) {
        return new WaitCommand(0.8).andThen(new SetDrive(drive, -0.5, -0.5).withTimeout(3));
    }

    public static CommandBase shootBackward(Drive drive, PneumaticCatapult catapult) {
        InstantCommand launch = new InstantCommand(catapult::launch, catapult);
        InstantCommand retract = new InstantCommand(catapult::retract, catapult);
        return launch.andThen(new WaitCommand(0.5)).andThen(retract).andThen(new WaitCommand(0.5)).andThen(new SetDrive(drive, 0.5, 0.5).withTimeout(5));
    }

    public static CommandBase shootOnly(PneumaticCatapult catapult) {
        InstantCommand launch = new InstantCommand(catapult::launch, catapult);
        return launch;
    }

    public static CommandBase pushDrive(Drive drive) {
        return new SetDrive(drive, 0.5, 0.5).withTimeout(0.75).andThen(new SetDrive(drive, -0.5, -0.5).withTimeout(5));
    }
}
