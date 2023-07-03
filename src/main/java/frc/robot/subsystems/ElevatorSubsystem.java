// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  CANSparkMax elev_Motor = new CANSparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);
  RelativeEncoder elev_Encoder = elev_Motor.getEncoder();
  PIDController elev_pidController = new PIDController(0.4, 0, 0);
  double desired_position = 0;

  //Setting Positions
  public void setPosition0(){
    desired_position = 0;
  }

  public void setPositionLow(){
    desired_position = -2.255;
  }

  public void setPositionMid(){
    desired_position = -6.6;
  }

  public void setPositionHigh(){
    desired_position = -10.1;
  }

  //Reset Encoders
  public void resetEncoders(){
    elev_Encoder.setPosition(0.0);
  }

  //Stops motors
  public void stop(){
    desired_position = elev_Encoder.getPosition();
  }

  public void ManualControl (double JoystickInput){
    desired_position = JoystickInput * 0.01 + desired_position;
  }

  public boolean isFinished(){
    if (MathUtil.clamp(elev_pidController.calculate(elev_Encoder.getPosition(), desired_position), -0.5, 0.5) < 0.02)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  public ElevatorSubsystem() {}


  @Override
  public void periodic() {
    
    elev_pidController.setIntegratorRange(-0.6, 0.6);
    elev_Motor.set(MathUtil.clamp(elev_pidController.calculate(elev_Encoder.getPosition(), desired_position), -0.5, 0.5));

  }
}
