package robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    // Constructor
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

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

        resetEncoders();
    }

    public double getDrivePosition() {
        return (driveMotor.getSelectedSensorPosition() * SwerveModuleConstants.kDriveEncoderDistancePerUnitMeters);
    }

    public double getDriveVelocity() {
        double vel = driveMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        vel = vel * 10.0 * SwerveModuleConstants.kDriveEncoderDistancePerUnitMeters;  // This is the velocity in Meters Per Second
        return vel;
    }
    
    //public double getTurningPosition() {
    //    // ??? What is the point of this ... The motor has gear reduction and is NOT Absolute ???
    //    return turningEncoder.getPosition();
    //}

    public double getTurningVelocityDegrees() {
        double rotVel = turningMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        rotVel = rotVel * 10.0 * SwerveModuleConstants.kTurningEncoderDegreesPerEncoderCount;  // This is the velocity in Degrees Per Second
        return rotVel;
    }

    public double getTurningVelocityRadians() {
        double rotVel = turningMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        rotVel = rotVel * 10.0 * SwerveModuleConstants.kTurningEncoderRadiansPerEncoderCount;  // This is the velocity in Radians Per Second
        return rotVel;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition();            // Position in Radians
        angle -= absoluteEncoderOffsetRad;                       // Correct for Sensor Misaligned
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);  // Change sign as needed
        return angle;
    }

    public double getAbsoluteEncoderDegrees() {
        double angle = absoluteEncoder.getPosition();            // Position in Radians
        angle -= absoluteEncoderOffsetRad;                       // Correct for Sensor Misaligned
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);  // Change sign as needed
        angle = Math.toDegrees(angle);                           // Change to Degrees 
        return angle;
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);        // Set Drive Motor Distance to Zero
        turningMotor.setSelectedSensorPosition(0);      // Not really Needed ????
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + "absoluteEncoderID" + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
