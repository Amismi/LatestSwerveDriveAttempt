package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubystem extends SubsystemBase{
    //intialize gryo
    private AHRS gryo = new AHRS(SPI.Port.kMXP);

    //creating every swerve module giving the parameters from the java.constant file
    private final SwerveModule frontLeft = new SwerveModule(
        driveConstants.kFrontLeftDriveMotorPort,
        driveConstants.kFrontLeftAngleMotorPort,
        driveConstants.kFrontLeftDriveEncoderReversed,
        driveConstants.kFrontLeftAngleEncoderReversed,
        driveConstants.kFrontLeftAbsoluteAngleEncoderPort,
        driveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        driveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );
    private final SwerveModule frontRight = new SwerveModule(
        driveConstants.kFrontRightDriveMotorPort,
        driveConstants.kFrontRightAngleMotorPort,
        driveConstants.kFrontRightDriveEncoderReversed,
        driveConstants.kFrontRightAngleEncoderReversed,
        driveConstants.kFrontRightAbsoluteAngleEncoderPort,
        driveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        driveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );
    private final SwerveModule backLeft = new SwerveModule(
        driveConstants.kBackLeftDriveMotorPort,
        driveConstants.kBackLeftAngleMotorPort,
        driveConstants.kBackLeftDriveEncoderReversed,
        driveConstants.kBackLeftAngleEncoderReversed,
        driveConstants.kBackLeftAbsoluteAngleEncoderPort,
        driveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        driveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );
    private final SwerveModule backRight = new SwerveModule(
        driveConstants.kBackRightDriveMotorPort,
        driveConstants.kBackRightAngleMotorPort,
        driveConstants.kBackRightDriveEncoderReversed,
        driveConstants.kBackRightAngleEncoderReversed,
        driveConstants.kBackRightAbsoluteAngleEncoderPort,
        driveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        driveConstants.kBackRightDriveAbsoluteEncoderReversed
    );

    //Function to reset gyro for every time the robot boots up to make sure that the forward direction is the default value
    
    public SwerveSubystem()
    {
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch (Exception e) {
            }
        }).start();
    }
    //function to reset gyro
    public void zeroHeading()
    {
        gryo.reset();
    }

    //function to get the direction that we are facing
    public double getHeading()
    {
        return Math.IEEEremainder(gryo.getAngle(), 360); //this function also makes the gryo values not go over 360 to be easier to read
    }
    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getHeading()); //changes the 360 gyro values to degrees
    }
    //sending to dashboard on computer for debugging 
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Robot Direction", getHeading());
    }
    // creating a function to stop the modules
    public void stopModules()
    {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    //creating a function to set the module states with parameters on an array and normalizing speeds
    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, driveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        frontLeft.setDesiredState(desiredStates[2]);
        frontLeft.setDesiredState(desiredStates[3]);
    }

}
 