// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.PoleCatCommand;
// import frc.robot.commands.PolecatPID;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PneumaticCatapult;
import frc.robot.subsystems.Polecat;
import frc.robot.subsystems.Polecat.Direction;

public class RobotContainer {
  private final Drive drive = new Drive();
  public final PneumaticCatapult catapult = new PneumaticCatapult();
  private final Polecat polecat = new Polecat();

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.controllerPort);

  public RobotContainer() {
    configureBindings();
    CameraServer.startAutomaticCapture();
  }

  private void configureBindings() {
    // Pneumatic catapult
    InstantCommand launch = new InstantCommand(catapult::launch, catapult);
    InstantCommand retract = new InstantCommand(catapult::retract, catapult);
    driverController.a().onTrue(launch.andThen(Commands.waitSeconds(0.5)).andThen(retract));


    // Polecat
    // PoleCatCommand poleUp = new PoleCatCommand(polecat,0.4);
    // PoleCatCommand poleDown = new PoleCatCommand(polecat,-0.4);
    PoleCatCommand slowPoleUp = new PoleCatCommand(polecat, 0.3);
    PoleCatCommand slowPoleDown = new PoleCatCommand(polecat, -0.3);
    // PolecatPID poleUp = new PolecatPID(polecat,0);
    // PolecatPID poleDown = new PolecatPID(polecat,0);
    driverController.x().whileTrue(slowPoleUp);
    driverController.y().whileTrue(slowPoleDown);
    driverController.b().onTrue(slowPoleDown.withTimeout(0.3).andThen(slowPoleUp.withTimeout(0.1)));
    // driverController.x().onTrue(poleDown).onFalse(poleUp);

    // InstantCommand go = new InstantCommand(drive::setAll, drive);
    // InstantCommand stop = new InstantCommand(drive::zero, drive);
    // driverController.leftBumper().onTrue(go).onFalse(stop);

    // Drive
    ArcadeDrive arcadeDrive = new ArcadeDrive(drive, driverController::getLeftY, driverController::getLeftX);
    // TankDrive tankDrive = new TankDrive(drive, driverController::getLeftY, driverController::getRightY);
    drive.setDefaultCommand(arcadeDrive);

    InstantCommand defaultDrive = new InstantCommand(() -> drive.setSpeed(OperatorConstants.defaultThrottle,OperatorConstants.defaultRotate));
    InstantCommand slowDrive = new InstantCommand(() -> drive.setSpeed(OperatorConstants.slowThrottle,OperatorConstants.slowRotate));
    driverController.leftBumper().whileTrue(slowDrive).whileFalse(defaultDrive);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
