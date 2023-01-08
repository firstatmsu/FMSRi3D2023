// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drive;

public class ArcadeDrive extends CommandBase {
  private final Drive drive;

  private final Supplier<Double> throttleStick;
  private final Supplier<Double> rotateStick;

  public ArcadeDrive(Drive drive, Supplier<Double> leftStick, Supplier<Double> rightStick) {
    this.drive = drive;
    this.throttleStick = leftStick;
    this.rotateStick = rightStick;

    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double throttleSpeed = throttleStick.get() * OperatorConstants.throttleMultiplier;
    double rotateSpeed = rotateStick.get() * OperatorConstants.rotateMultiplier;

    drive.arcadeDrive(throttleSpeed, rotateSpeed);;
  }

  @Override
  public void end(boolean interrupted) {
    drive.zero();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
