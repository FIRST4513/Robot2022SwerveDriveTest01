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

    // -------- Various Constants ------------------
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);      // 4 Inch wheel
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kDriveEncoderCountsPerRev = 4096;

    // ****************************************************************
    //
    //      4096 Motor Encoder Counts per MOTOR revolution
    //    * 6.75 gear ratio - Motor revolutions to 1 wheel revolution
    //  --------
    //  = 27,648 Motor Encoder Counts Per WHEEL Revolution
    // 
    //      4.00  Inches Wheel Diameter
    //     12.56 Inches Wheel Circumfrence
    //      0.319024 Meters Wheel Circumfrence
    //   / 27648 Divide by Motor Encoder Counts Per Wheel Rotation
    //  --------
    //   = 0.0000115387731 Meters Traveled Per Motor Encoder Count
    //
    // ****************************************************************

    public static final double kDriveEncoderDistancePerUnitMeters = 0.0000115387731;

    //public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;  // ?????????????

    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kTurningMotorEncoderCountsPerRev = 4096;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;   // ?????????????
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;  // ?????????????

    // PID Constant
    public static final double kPTurning = 0.5;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        // Drive Motor Config
        driveMotor      = new WPI_TalonFX(driveMotorId);
        driveMotor.setInverted(driveMotorReversed);

        // Turning Motor Config
        turningMotor    = new WPI_TalonFX(turningMotorId);
        turningMotor.setInverted(turningMotorReversed);

        // CANCoder Absolute Turning Encoder Config
        // Set units of the CANCoder to radians and velocity being radians per second
        absoluteEncoder = new CANCoder(absoluteEncoderId);
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString= "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        absoluteEncoder.configAllSettings(config);

        // Set Up PID Controller
        turningPidController = new PIDController(kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return (driveMotor.getSelectedSensorPosition() * kDriveEncoderDistancePerUnitMeters);
    }

    public double getDriveVelocity() {
        double vel = driveMotor.getSelectedSensorVelocity();    // This is the counts for the last 100ms
        vel = vel * 10.0 * kDriveEncoderDistancePerUnitMeters;  // This is the velocity in Meters Per Second
        return vel;
    }
    
    //public double getTurningPosition() {
    //    // ??? What is the point of this ... The motor has gear reduction and is NOT Absolute ???
    //    return turningEncoder.getPosition();
    //}

    //public double getTurningVelocity() {
    //    // ??? What is the point of this ... The motor has gear reduction and is NOT Absolute ??? 
    //    return turningEncoder.getVelocity();
    //}

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition();           // Position in Radians
        angle -= absoluteEncoderOffsetRad;                      // Correct for Sensor Misaligned
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);  // Change sign as needed
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
