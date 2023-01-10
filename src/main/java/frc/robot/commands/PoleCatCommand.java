// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Polecat;
import frc.robot.subsystems.Polecat.Direction;


public class PoleCatCommand extends CommandBase {
  /**
   * Creates a new RunConveyorCommand.
   */
  private final Polecat polecat;
  private final double speed; 

  public PoleCatCommand(Polecat polecat, double speed) {
    this.polecat = polecat;
    this.speed = speed;
    addRequirements(polecat);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // switch (direction) {
    //   case kUp:
    //     polecat.movePoleCat();
    //     break;
    //   case kDown:
    //     polecat.movePoleCat(-OperatorConstants.poleCatSpeed);
    //     break;
    //   default:
    //     break;
    // }
    polecat.movePoleCat(-speed);
  }

  @Override
  public boolean isFinished() {
    return false;
    // return polecat.isLimited(direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    polecat.stopPoleCat();
  }

}
