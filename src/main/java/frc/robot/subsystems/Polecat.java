// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.IdentifierConstants;

public class Polecat extends SubsystemBase {
  /** Creates a new Polecat. */
  protected final DigitalInput upperLimitSwitch;
  protected final DigitalInput lowerLimitSwitch;

  // private final CANSparkMax poleCat;
  private final VictorSPX poleCat;
  // private final RelativeEncoder encoder;

  public Polecat() {
    // poleCat = new CANSparkMax(IdentifierConstants.polecat, MotorType.kBrushless);
    poleCat = new VictorSPX(IdentifierConstants.polecat);
    // encoder = poleCat.getEncoder();
    // encoder.setPositionConversionFactor(EncoderConstants.poleCat);
    // encoder.setPosition(0);

    upperLimitSwitch = new DigitalInput(IdentifierConstants.polecatUpperSwitch);
    lowerLimitSwitch = new DigitalInput(IdentifierConstants.polecatLowerSwitch);
  }

  public enum Direction {
    kUp,
    kDown,
  }

  // public double getPosition() {
  //   return encoder.getPosition();
  // }

  public void movePoleCat(double speed) {
    // poleCat.set(speed);
    poleCat.set(ControlMode.PercentOutput, speed);
  }

  public boolean isLimited(Direction direction) {
    return (direction == Direction.kUp) ? upperLimitSwitch.get() : lowerLimitSwitch.get();
  }

  public void stopPoleCat() {
    // poleCat.stopMotor();
    poleCat.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("PolecatPosition", Math.toDegrees(getPosition()));
  }

}
