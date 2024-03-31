package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.driveConstants;


public class SwerveModule {
    //Swerve module motor variables
    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private CANCoder Encoder;    
    

    // Swerve Module encoder variables
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;

    // intialize PID
    private PIDController turningPidController;

    //analog input encoder used to keep wheel turn position even after robo turn off
    private AnalogInput absoluteEncoder;
    private boolean isAbsoluteEncoderReversed;
    private double absoluteEncoderOffsetRadians;

    public SwerveModule(int driveMotorId, int angleMotorId, 
        boolean isDriveMotorReversed, boolean isTurningMotorReversed, int absoluteEncoderId,
            double absoluteEncoderOffset, boolean absoluteEncoderReversed)
    {
        //sets the encoder offsets and reversed status and ids
        this.absoluteEncoderOffsetRadians = absoluteEncoderOffset;
        this.isAbsoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = AnalogInput(absoluteEncoderId);
        //set motor for each module
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorId, MotorType.kBrushless);
        //set encoder for each module
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        //switching from meters to radians and meters
        driveEncoder.setPositionConversionFactor(SwerveConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.kDriveEncoderRPM2MeterPerSec);
        angleEncoder.setPositionConversionFactor(SwerveConstants.kAngleEncoderRot2Radians);
        angleEncoder.setVelocityConversionFactor(SwerveConstants.kAngleEncoderRPM2RadPerSec);
        //intialize PID Controller
        turningPidController = new PIDController(SwerveConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        //reset encoders upon intialization
        resetEncoder();

        
    }
    //getting encoder values
    public double getDrivePosition()
    {
        return driveEncoder.getPosition();
    }
    public double getAnglePosition()
    {
        return angleEncoder.getPosition();
    }
    public double getDriveVelocity()
    {
        return driveEncoder.getVelocity();
    }
    public double getAngleVelocity()
    {
        return angleEncoder.getVelocity();
    }
    public double getAbsoluteEncoderRad()
    {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    private AnalogInput AnalogInput(int absoluteEncoderId) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'AnalogInput'");
    }
    
    //function to reset encoders to absolute encoders
    public void resetEncoder()
    {
        driveEncoder.setPosition(0);
        angleEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
    }
    public void setDesiredState(SwerveModuleState state)
    {
        if (Math.abs(state.speedMetersPerSecond) < 0.001)
        {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / driveConstants.kMaxSpeedMetersPerSecond);
        angleMotor.set(turningPidController.calculate(getAnglePosition(), state.angle.getRadians()));

    }
    public void stop()
    {
        driveMotor.set(0);
        angleMotor.set(0);
    }



}
