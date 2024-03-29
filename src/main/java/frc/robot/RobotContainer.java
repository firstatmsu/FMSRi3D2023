// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autons;
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
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.controllerPort);

  private final CommandXboxController opController = 
      new CommandXboxController(OperatorConstants.operatorPort);

  public RobotContainer() {
    configureBindings();
    CameraServer.startAutomaticCapture();
  }

  public void robotInit() {
    chooser.addOption("Drive Forwards", Autons.forwardAuton(drive));
    chooser.setDefaultOption("Shoot, Drive Backwards", Autons.shootBackward(drive, catapult));
    chooser.addOption("Only Shoot", Autons.shootOnly(catapult));
    chooser.addOption("Push Drive", Autons.pushDrive(drive));
    SmartDashboard.putData("Choose Auton", chooser);
  }

  private void configureBindings() {
    // Pneumatic catapult
    InstantCommand launch = new InstantCommand(catapult::launch, catapult);
    InstantCommand retract = new InstantCommand(catapult::retract, catapult);
    opController.a().onTrue(launch.andThen(Commands.waitSeconds(0.5)).andThen(retract));


    // Polecat
    // PoleCatCommand poleUp = new PoleCatCommand(polecat,0.4);
    // PoleCatCommand poleDown = new PoleCatCommand(polecat,-0.4);
    PoleCatCommand slowPoleUp = new PoleCatCommand(polecat, 0.4);
    PoleCatCommand slowPoleDown = new PoleCatCommand(polecat, -0.3);
    PoleCatCommand timedPoleUp = new PoleCatCommand(polecat, 0.3);
    PoleCatCommand timedPoleDown = new PoleCatCommand(polecat, -0.3);
    // PolecatPID poleUp = new PolecatPID(polecat,0);
    // PolecatPID poleDown = new PolecatPID(polecat,0);
    opController.x().whileTrue(slowPoleUp);
    opController.y().whileTrue(slowPoleDown);
    opController.b().onTrue(timedPoleDown.withTimeout(0.4));
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
    return new WaitCommand(4).andThen(chooser.getSelected());
  }
}
