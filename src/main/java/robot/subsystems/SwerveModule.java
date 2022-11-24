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


    // --------- Absolute Encoder Methods -----------
    public double getTurningVelRadians() {
        double rotVel = turningMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        // This is the velocity in Radians Per Second
        return ((rotVel * 10.0) * SwerveModuleConstants.kTurningEncoderRadiansPerEncoderCount); 
    }

    public double getTurningVelDegrees() {
        double rotVel = turningMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
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

    // -------------- Swerve Module STATE Methods -----------

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelMeters(), new Rotation2d(getWheelAngleRadians()));
    }

    public void setDesiredState(SwerveModuleState state) {
        // Passed state provides - Wheel Velocity in Meters/Sec and Wheel Angle in Radians.
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            // If the requested speed is too low just stop the motors and get out.
            stop(); 
            return;
        }
        // Compare the current Wheel Angle to Target and determine shortest route
        state = SwerveModuleState.optimize(state, getState().angle);
        // Power the Drive Motor (This converts a command in meters/sec into -1.0 to +1.0)
        driveMotor.set(state.speedMetersPerSecond / DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond);
        // Power the Turning Motor - This uses a PID controller to lock in on Angle (Current Angle , Setpoint)
        turningMotor.set(turningPidController.calculate(getWheelAngleRadians(), state.angle.getRadians()));
        // Update smart dashboard
        SmartDashboard.putString( swerveModuleID + " State", state.toString());
        SmartDashboard.putNumber( swerveModuleID + " State Angle Degrees", state.angle.getDegrees());
        SmartDashboard.putNumber( swerveModuleID + " State Speed Meters", state.speedMetersPerSecond);
        SmartDashboard.putNumber( swerveModuleID + " Dist Meters", getDriveDistMeters());
        SmartDashboard.putNumber( swerveModuleID + " Dist Inches", getDriveDistInches());
        SmartDashboard.putNumber( swerveModuleID + " Vel Meters/Sec", getDriveVelMeters());
        SmartDashboard.putNumber( swerveModuleID + " Vel Ft/Sec", getDriveVelInches()/12.0);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
