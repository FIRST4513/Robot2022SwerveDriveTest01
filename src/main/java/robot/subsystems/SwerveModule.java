package robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import robot.Constants.DriveTrainConstants;
import robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    // ---------------- Various Devices ------------------
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;
    private final CANCoder absoluteEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final String swerveModuleID;

    // Constructor
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
                        boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset,
                        boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        if (driveMotorId == DriveTrainConstants.kFrontLeftDriveMotorPort){
            this.swerveModuleID = "Front Left";
        } else if (driveMotorId == DriveTrainConstants.kBackLeftDriveMotorPort){ 
            this.swerveModuleID = "Back Left";
        } else if (driveMotorId == DriveTrainConstants.kFrontRightDriveMotorPort){ 
            this.swerveModuleID = "Front Right";
        } else if (driveMotorId == DriveTrainConstants.kBackRightDriveMotorPort){ 
            this.swerveModuleID = "Back Right";
        } else { 
            this.swerveModuleID = "error";
        }

        // Drive Motor Config
        driveMotor = new WPI_TalonFX(driveMotorId);
        driveMotor.setInverted(driveMotorReversed);

        // Turning Motor Config
        turningMotor = new WPI_TalonFX(turningMotorId);
        turningMotor.setInverted(turningMotorReversed);

        // CANCoder Absolute Turning Encoder Config
        // Set units of the CANCoder to radians and velocity being radians per second
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        CANCoderConfiguration config = new CANCoderConfiguration(); 
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString= "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        absoluteEncoder.configAllSettings(config);  // Send the config to the Encoder

        // Set Up PID Controller
        turningPidController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetDriveEncoder();
    }


    // --------- Drive Encoder Methods -----------
    public double getDriveEncoderRaw(){
        return driveMotor.getSelectedSensorPosition();
    }

    public double getDriveDistMeters() {
        return (driveMotor.getSelectedSensorPosition() * 
                    SwerveModuleConstants.kDriveEncoderDistancePerUnitMeters);
    }
    public double getDriveDistInches() {
        return (driveMotor.getSelectedSensorPosition() *
                    SwerveModuleConstants.kDriveEncoderDistancePerUnitInches);
    }
    
    public double getDriveVelMeters() {
        double vel = driveMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        // This is the velocity in Meters Per Second
        return ((vel * 10.0) * SwerveModuleConstants.kDriveEncoderDistancePerUnitMeters);  
    }
    
    public double getDriveVelInches() {
        double vel = driveMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        // This is the velocity in Inches Per Second
        return ((vel * 10.0) * SwerveModuleConstants.kDriveEncoderDistancePerUnitInches);
    }
    public void resetDriveEncoder() {
        driveMotor.setSelectedSensorPosition(0);        // Set Drive Motor Distance to Zero
    }


    // --------------------- Absolute Encoder Methods ----------------------
    // Wheel angles need to reported + for CCW and - for CW Rotation
    public double getTurningVelRadians() {
        double rotVel = turningMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        rotVel = rotVel * (absoluteEncoderReversed ? -1.0 : 1.0);    // Change sign as needed
        // This is the velocity in Radians Per Second
        return ((rotVel * 10.0) * SwerveModuleConstants.kTurningEncoderRadiansPerEncoderCount); 
    }

    public double getTurningVelDegrees() {
        double rotVel = turningMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        rotVel = rotVel * (absoluteEncoderReversed ? -1.0 : 1.0);    // Change sign as needed
        // This is the velocity in Degrees Per Second
        return ((rotVel * 10.0) * SwerveModuleConstants.kTurningEncoderDegreesPerEncoderCount); 
    }

    public double getWheelAngleRadians() {
        double angle = absoluteEncoder.getPosition();            // Position in Radians
        angle -= absoluteEncoderOffsetRad;                       // Correct for Sensor Misaligned
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);  // Change sign as needed
        return angle;
    }

    public double getWheelAngleDegrees() {
        // Degrees 0 to +-180 Degrees
        double angle = absoluteEncoder.getPosition();            // Position in Radians
        angle -= absoluteEncoderOffsetRad;                       // Correct for Sensor Misaligned
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);  // Change sign as needed
        angle = Math.toDegrees(angle);                           // Change to Degrees 
        return angle;
    }

    public double getWheelAngleRaw(){
        // Position in Radians
        return absoluteEncoder.getPosition();
    }    

    // -------------- Drive Swerve Module Motors -----------
    public void setMotorsState(SwerveModuleState state) {
        // State (Wheel Velocity in Meters/Sec and Wheel Angle in Radians).

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            // Speed is too low just stop the motors.
            stopMotors(); 
            return;
        }
        // Optimized for shortest turn route
        state = SwerveModuleState.optimize(state, getMotorsState().angle);

        // Power the Drive Motor (This converts a command in meters/sec into -1.0 to +1.0)
        driveMotor.set(state.speedMetersPerSecond / DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond);

        // Power the Turning Motor - (This uses a PID controller to lock in on Angle (Current Angle , Setpoint))
        turningMotor.set(turningPidController.calculate(getWheelAngleRadians(), state.angle.getRadians()));
    }

    public void stopMotors() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public SwerveModuleState getMotorsState() {
        return new SwerveModuleState(getDriveVelMeters(), new Rotation2d(getWheelAngleRadians()));
    }

    public void updateShuffleboard(){
        SmartDashboard.putString( swerveModuleID + " State", getMotorsState().toString());
        SmartDashboard.putNumber( swerveModuleID + " State Angle Degrees", getMotorsState().angle.getDegrees());
        SmartDashboard.putNumber( swerveModuleID + " State Speed Meters", getMotorsState().speedMetersPerSecond);

        SmartDashboard.putNumber( swerveModuleID + " Wheel Angle Raw",      getWheelAngleRaw());
        SmartDashboard.putNumber( swerveModuleID + " Wheel Angle Degrees",  getWheelAngleDegrees());
        SmartDashboard.putNumber( swerveModuleID + " Drive Dist Raw",       getDriveEncoderRaw());
        SmartDashboard.putNumber( swerveModuleID + " Drive Dist Inches",    getDriveDistInches());
        SmartDashboard.putNumber( swerveModuleID + " Drive Dist Meters",    getDriveDistMeters());
        SmartDashboard.putNumber( swerveModuleID + " Drive Vel Ft/Sec",     getDriveVelInches()/12.0);
        SmartDashboard.putNumber( swerveModuleID + " Drive Vel Meters/Sec", getDriveVelMeters());

    }
}
