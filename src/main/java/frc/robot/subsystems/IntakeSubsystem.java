// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
  double speed = 0;

  public void motorFoward(){
    speed = IntakeConstants.intakeForwardSpeed;
  }

  public void motorReversed(){
    speed = IntakeConstants.intakeReverseSpeed;
  }

  public void motorZero(){
    speed = 0;
  }

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
}
