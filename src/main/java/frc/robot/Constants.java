// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class ModuleConstants{
    
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1/21.30;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kDriveMotorPositionConversionFactor = kDriveMotorGearRatio/2048*kWheelDiameterMeters*Math.PI;
    public static final double kDriveMotorVelocityConversionFactor = kDriveMotorGearRatio/2048*kWheelDiameterMeters*Math.PI*10;
}

public static final class DriveConstants{

  public static final double kTrackWidth = Units.inchesToMeters(28);
  // Distance between right and left wheels
  public static final double kWheelBase = Units.inchesToMeters(28);
  // Distance between front and back wheels
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
         
          );

  public static final int kFrontLeftDriveMotorPort = 40;
  public static final int kFrontRightDriveMotorPort = 10;
  public static final int kBackLeftDriveMotorPort = 46;
  public static final int kBackRightDriveMotorPort = 47;

  public static final int kFrontLeftTurningMotorPort = 14;
  public static final int kFrontRightTurningMotorPort = 5;
  public static final int kBackLeftTurningMotorPort = 36;
  public static final int kBackRightTurningMotorPort = 6;

  public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
  public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
  public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
  public static final int kBackRightDriveAbsoluteEncoderPort = 3;

  public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
  public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
  public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
  public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

  public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegrees = 56.9074;
  public static final double kFrontRightDriveAbsoluteEncoderOffsetDegrees = -38.06875; 
  public static final double kBackLeftDriveAbsoluteEncoderOffsetDegrees = 15.63875;
  public static final double kBackRightDriveAbsoluteEncoderOffsetDegrees = -150.42;

  public static final boolean kFrontLeftDriveMotorReversed = false;
  public static final boolean kFrontRightDriveMotorReversed = false;
  public static final boolean kBackLeftDriveMotorReversed = false;
  public static final boolean kBackRightDriveMotorReversed = false;

  public static final boolean kFrontLeftTurningMotorReversed = true;
  public static final boolean kFrontRightTurningMotorReversed = true;
  public static final boolean kBackLeftTurningMotorReversed = true;
  public static final boolean kBackRightTurningMotorReversed = true;

  public static final double kPhysicalMaxSpeedMetersPerSecond = 4.96824;
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

  public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.5;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 0.5;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}
  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
            DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 9;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, 
        kMaxAngularAccelerationRadiansPerSecondSquared);

    //Auto PID Controllers
    //X Controller
    public static final double xControllerP = 1.5;
    public static final double xControllerI = 0;
    public static final double xControllerD = 0.05;
   
    //Y Controller
    public static final double yControllerP = 1.5;
    public static final double yControllerI = 0;
    public static final double yControllerD = 0.05;

    //Theta Controller
    public static final double thetaControllerP = 3;
    public static final double thetaControllerI = 0;
    public static final double thetaControllerD = 0;

  
  }

  public static final class ClawConstants {
    public static final int forwardChannel = 1;
    public static final int reverseChannel = 0;
  }

  public static final class IntakeConstants {
    public static final int intakeMotorID = 58;
    public static final double intakeForwardSpeed = 0.5;
    public static final double intakeReverseSpeed = -0.5;
  }

  public static final class ElevatorConstants {
    public static final int elevatorMotorID = 57;
  }

  public static final class ArmConstants{
    public static final int armMotorID = 56; 
  }
}
