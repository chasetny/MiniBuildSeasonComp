package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class SwerveModule {
    
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;

    public SwerveModule(
        int driveMotorID, 
        int turningMotorID, 
        int absoluteEncoderID, 
        double absoluteEncoderOffset,
        boolean absoluteEncoderReversed,
        boolean driveMotorReversed,
        boolean turningMotorReversed){

        absoluteEncoder = new CANCoder(absoluteEncoderID);

        //Initializing motors and encoder
        driveMotor = new TalonFX(driveMotorID);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        turningEncoder = turningMotor.getEncoder();

        //Setting Conversion Factors
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        //Pid
        turningPidController = new PIDController(0.5, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        //Configuring parts
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configMagnetOffset(absoluteEncoderOffset);
        absoluteEncoder.configSensorDirection(absoluteEncoderReversed);
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        //Inverse Motors?
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
       
        new Thread(() -> {
            try {
                Thread.sleep(100);
                resetEncoders();;
            } catch (Exception e) {
            }
        }).start();
    }

    public double getDrivePosition(){
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveMotorPositionConversionFactor;
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveMotorVelocityConversionFactor;
    } 

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
       return absoluteEncoder.getAbsolutePosition()*Math.PI/180;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveMotorPositionConversionFactor, new Rotation2d (turningEncoder.getPosition()));
    }

    public void resetEncoders(){
        driveMotor.setSelectedSensorPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putNumber("current angle", getTurningPosition());
        SmartDashboard.putNumber("desiredAngle", state.angle.getRadians());
        SmartDashboard.putNumber("TurningMotor output", turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putNumber("driveMotor output", state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    public void stop(){
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(0);
    }
}

