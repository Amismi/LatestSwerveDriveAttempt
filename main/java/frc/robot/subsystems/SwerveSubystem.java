package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
public class SwerveSubystem extends SubsystemBase{
    private final SwerveModule frontLeft = SwerveModule(
        driveConstants.kFrontLeftDriveMotorPort,
        driveConstants.kFrontLeftAngleMotorPort,
        driveConstants.kFrontLeftDriveEncoderReversed,
        driveConstants.kFrontLeftAngleEncoderReversed,
        driveConstants.kFrontLeftAbsoluteAngleEncoderPort,
        driveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        driveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    
}
 