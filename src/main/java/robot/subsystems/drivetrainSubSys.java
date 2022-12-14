// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import robot.Constants.DriveTrainConstants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class drivetrainSubSys extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveTrainConstants.kFrontLeftDriveMotorPort,
        DriveTrainConstants.kFrontLeftTurningMotorPort,
        DriveTrainConstants.kFrontLeftDriveEncoderReversed,
        DriveTrainConstants.kFrontLeftTurningEncoderReversed,
        DriveTrainConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveTrainConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveTrainConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveTrainConstants.kFrontRightDriveMotorPort,
        DriveTrainConstants.kFrontRightTurningMotorPort,
        DriveTrainConstants.kFrontRightDriveEncoderReversed,
        DriveTrainConstants.kFrontRightTurningEncoderReversed,
        DriveTrainConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveTrainConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveTrainConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveTrainConstants.kBackLeftDriveMotorPort,
        DriveTrainConstants.kBackLeftTurningMotorPort,
        DriveTrainConstants.kBackLeftDriveEncoderReversed,
        DriveTrainConstants.kBackLeftTurningEncoderReversed,
        DriveTrainConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveTrainConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveTrainConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveTrainConstants.kBackRightDriveMotorPort,
        DriveTrainConstants.kBackRightTurningMotorPort,
        DriveTrainConstants.kBackRightDriveEncoderReversed,
        DriveTrainConstants.kBackRightTurningEncoderReversed,
        DriveTrainConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveTrainConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveTrainConstants.kBackRightDriveAbsoluteEncoderReversed);

    private ChassisSpeeds chassisSpeeds;        // Used to track and report to shuffleboard current State

    private int displayCtr = 0;

    // Gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Odometry
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry( 
        DriveTrainConstants.kDriveKinematics,
        new Rotation2d(0));

    public drivetrainSubSys() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetGyro();
            } catch (Exception e) {
            }
        }).start();
    }

    @Override
    public void periodic() {
        odometer.update(getGyroHeadingRotation2d(), 
                        frontLeft.getMotorsState(),
                        frontRight.getMotorsState(),
                        backLeft.getMotorsState(),
                        backRight.getMotorsState());
        if ( displayCtr % 10 == 0) updateShuffleBoard();      // Update shuufleBoard diplay every 200 ms
        displayCtr++;
    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // ---------------------------------- Gyro Methods ----------------------------------
    // Gyro hedings need to be - for CW and + for CCW
    public double getGyroHeadingDegrees() {
        // Returns total ACCUMULATED Z axis Angle ie) -xxxxxx degree to +xxxxxx degrees
        // Not limited to +-360, rolls over as turns continue
        // Returns angle +CW, -CCW .... This needs to be inverted for Kinematics
        double angle =  gyro.getAngle();
        if (DriveTrainConstants.invertGyro) angle *= -1; // Invert to CW(0 to -180)  CCW(0 to +180)
        return angle;
    
        //return Math.IEEEremainder(gyro.getAngle();, 360);  // Recalculates to -180 to +180 YAW degrees
    }

    public double getGyroYaw(){
        // returns Angle -180 to + 180 degrees
        double yaw = gyro.getYaw();
        if (DriveTrainConstants.invertGyro) yaw *= -1; // Invert to CW(0 to -180)  CCW(0 to +180)
        return yaw;
    }

    public Rotation2d getGyroHeadingRotation2d() {
        // Return a Rotation2d object of the yaw value -180 to +180 degree
        // I beleive the Rotation2d is stored as Radians !
        // Degrees -180 to +180 is then internaly converted to ( -PI to +PI )
        return Rotation2d.fromDegrees(getGyroYaw());
    }

    public void resetGyro() {
        gyro.reset();
    }

    // -------------------------------- Odometry Methods --------------------------------
    public Pose2d getPoseMeters() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getGyroHeadingRotation2d());
    }

    public void storeRobotChassisSpeed( ChassisSpeeds speeds){
        // This is only used to show in shuffleBoard
        chassisSpeeds = speeds; 
    }

    // ---------------------- Drive the Swerve Module Motors --------------------------
    public void setSwerveModulesStates(SwerveModuleState[] desiredStates) {
        // Make sure all wheels are at the same Velocity (in the "desiredStates" Array)
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, 
                DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond);
        // Send State (Velocity and Angle) to Each Swerve Drive Motor
        frontLeft.setMotorsState(desiredStates[0]);
        frontRight.setMotorsState(desiredStates[1]);
        backLeft.setMotorsState(desiredStates[2]);
        backRight.setMotorsState(desiredStates[3]);
    }

    public void stopSwerveMotors() {
        // Should this be done through setModuleStates method ?????
        frontLeft.stopMotors();
        frontRight.stopMotors();
        backLeft.stopMotors();
        backRight.stopMotors();
    }

    // --------------------------- Update ShuffleBoard -------------------------------
    private void updateShuffleBoard(){
        SmartDashboard.putNumber("Robot Gyro Hdg Degrees",  getGyroHeadingDegrees());
        SmartDashboard.putNumber("Robot Gyro Yaw",          getGyroYaw());
        SmartDashboard.putString("Robot Loc Meters",        getPoseMeters().getTranslation().toString());
        SmartDashboard.putNumber("Robot Loc X Ft",          Units.metersToFeet(getPoseMeters().getX()));
        SmartDashboard.putNumber("Robot Loc Y Ft",          Units.metersToFeet(getPoseMeters().getY()));
        SmartDashboard.putString("Robot Kinematics",        DriveTrainConstants.kDriveKinematics.toString());

        SmartDashboard.putNumber("Robot Fwd Vel Meters/Sec",  chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Robot Fwd Vel Ft/Sec",      Units.metersToFeet(chassisSpeeds.vxMetersPerSecond));
        SmartDashboard.putNumber("Robot Side Vel Meters/Sec", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Robot Side Vel Ft/Sec",     Units.metersToFeet(chassisSpeeds.vyMetersPerSecond));
        SmartDashboard.putNumber("Robot Rot Vel Degrees/Sec", Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));

        frontLeft.updateShuffleboard();
        frontRight.updateShuffleboard();
        backLeft.updateShuffleboard();
        backRight.updateShuffleboard();
    }

}


