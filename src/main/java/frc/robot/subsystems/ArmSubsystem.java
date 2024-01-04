// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  CANSparkMax arm_Motor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
  RelativeEncoder arm_Encoder = arm_Motor.getEncoder();
  PIDController arm_pidController = new PIDController(0.4, 0, 0);
  double desired_position = 0;
  

  public void setPosition0(){
    desired_position = 0.0;
  }

  public void setPositionMid(){
    desired_position = 5.2;
  }

  public void setPositionHigh(){
    desired_position = 5.2;
  }
  public void resetEncoders(){
    arm_Encoder.setPosition(0.0);
  }
  public void stop(){
    desired_position = arm_Encoder.getPosition();
  }
    public void ManualControl (double JoystickInput){
      desired_position = JoystickInput * 0.01 + desired_position;
  }

  public boolean isFinished(){
    if (MathUtil.clamp(arm_pidController.calculate(arm_Encoder.getPosition(), desired_position), -0.5, 0.5) < 0.02)
    {
      return true;
    }
    else
    {
      return false;
    }
  }




  public ArmSubsystem() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      arm_pidController.setIntegratorRange(-0.6, 0.6);
      arm_Motor.set(MathUtil.clamp(arm_pidController.calculate(arm_Encoder.getPosition(), desired_position), -0.5, 0.5));
  }
}
