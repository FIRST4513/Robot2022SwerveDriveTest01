// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    // ***************************** SwerveModule Constants **********************************
    public static final class SwerveModuleConstants {
        // Drive Wheel Constants
        public static final double kWheelDiameterInches = 4.0;        
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kWheelCircInches = 12.56;         
        public static final double kWheelCircMeters = Units.inchesToMeters(kWheelCircInches);
    
        // ------------------------  Drive Motor/Encoder Constants  ----------------------
        //
        //           4096 Motor Encoder Counts per MOTOR revolution
        //   /   0.014815 Gear Ratio is 1 / 6.75 = 0.14815 Wheel revolutions per Motor revolution
        //   ------------
        //       = 27,648 Motor Encoder Counts Per WHEEL Revolution
        // 
        //      4.00  Inches Wheel Diameter
        //     12.56 Inches Wheel Circumfrence
        //      0.319024 Meters Wheel Circumfrence
        //   / 27648 Divide by Motor Encoder Counts Per Wheel Rotation
        //  --------
        //   = 0.0000115387731 Meters Traveled Per Motor Encoder Count
        //
        // Note: inch * 0.0254 = meters.
        // Note: Meter * 39.3701 = Inches
        // Note: radians per Encoder Count (1 degree = pi/180 = 0.01745)
        //
        // ****************************************************************
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kDriveEncoderCountsPerMotorRev = 4096;
        public static final double kDriveEncoderCountsPerWheelRev = 
                    kDriveEncoderCountsPerMotorRev / kDriveMotorGearRatio;
        public static final double kDriveEncoderDistancePerUnitInches =
                    kWheelCircInches / kDriveEncoderCountsPerWheelRev;
        public static final double kDriveEncoderDistancePerUnitMeters =
                     Units.inchesToMeters(kDriveEncoderDistancePerUnitInches);    

        // -------------------  CANCoder Absolute Encoder Constants  ----------------------
        // This Encoder is 12 bit (4096)
        //
        //        360 degrees 
        //   /   4096 Divide by Absolute Encoder Counts Per WHEEL Rotation
        //  =========
        //  = 0.08789    degrees per Encoder Count
        //  = 0.00153367 radians per Encoder Count (1 degree = pi/180 = 0.01745)
        // 
        // ****************************************************************

        public static final double kTurningEncoderCountsPerRev = 4096;
        public static final double kTurningEncoderDegreesPerEncoderCount =
                    360 / kTurningEncoderCountsPerRev;
        public static final double kTurningEncoderRadiansPerEncoderCount =
                    Math.toRadians(kTurningEncoderDegreesPerEncoderCount);

        // ------------------------  PID Constant  ------------------------------
        public static final double kPTurning = 0.5;
    }

    // ***************************** DriveTrain Constants **********************************
    public static final class DriveTrainConstants {

        // Drivetrain Kinematics
        public static final double kTrackWidth = Units.inchesToMeters(23.75); // Between right/left wheels
        public static final double kWheelBase = Units.inchesToMeters(23.75);  // Between front/back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        // Front Left Swerve Drive Configs (CAN ID's and various flags and offsets)
        public static final int     kFrontLeftDriveMotorPort = 1;
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final int     kFrontLeftTurningMotorPort = 5;
        public static final boolean kFrontLeftTurningEncoderReversed = true;    // Not Used ????
        public static final int     kFrontLeftDriveAbsoluteEncoderPort = 9;

        public static final double  kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.019;
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

        // Front Right Swerve Drive
        public static final int     kFrontRightDriveMotorPort = 2;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final int     kFrontRightTurningMotorPort = 6;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final int     kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final double  kFrontRightDriveAbsoluteEncoderOffsetRad = -0.007;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

        // Back Left Swerve Drive
        public static final int     kBackLeftDriveMotorPort = 3;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final int     kBackLeftTurningMotorPort = 7;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final int     kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final double  kBackLeftDriveAbsoluteEncoderOffsetRad = -0.009;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

        // Back Right Swerve Drive
        public static final int     kBackRightDriveMotorPort = 4;
        public static final boolean kBackRightDriveEncoderReversed = false;
        public static final int     kBackRightTurningMotorPort = 8;
        public static final boolean kBackRightTurningEncoderReversed = true;
        public static final int     kBackRightDriveAbsoluteEncoderPort = 12;
        public static final double  kBackRightDriveAbsoluteEncoderOffsetRad = 0.026;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // The maximum velocity of the robot in meters per second.
        // This is a measure of how fast the robot should be able to drive in a straight line.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
        //
        //       6380.0  Falcon 500 RPM at free rotation
        //      /  6.75   MK4i Swerve Drive middle speed gear ratio
        //     ========
        //  =  945.1851 Max Wheel RPM
        //     /     60 Seconds 
        //    =========
        //  =    15.753 Wheel Revs Per Second
        //     *  12.56 Wheel Circumfrence (Inches) 
        //    =========
        //  =   197.858 Velocity per Second in Inches
        //   /       12 Inches / Foot
        //    =========
        //  =    16.488 Feet Per Second
        //  =    5.0256 Meters Per Second
        //
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // 2 Rev/Sec

        // Reduce the Max rates for Teleop - Slower for better Control ( 1/4 Too Much ?????)
        public static final double kTeleDriveThrottle = 0.25;       // Limit speeds to 1/4 Normal in teleop
        public static final double kTeleDriveMaxSpeedMetersPerSecond =
                kPhysicalMaxSpeedMetersPerSecond * kTeleDriveThrottle;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond * kTeleDriveThrottle;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    // ***************************** Autonomous Constants **********************************
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond =
                DriveTrainConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                DriveTrainConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    // ***************************** OI Constants **********************************
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverChassisOrientedButtonIdx = 2;
        public static final double kDeadband = 0.05;
    }
}

