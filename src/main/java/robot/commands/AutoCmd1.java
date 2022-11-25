package robot.commands;

import robot.Constants;
import robot.Constants.AutoConstants;
import robot.Constants.DriveTrainConstants;
import robot.subsystems.drivetrainSubSys;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class AutoCmd1 extends SequentialCommandGroup {
    public SequentialCommandGroup AutoCmd2(drivetrainSubSys subsystem ){

    // Step 1. Configure trajectory settings and add a Kinematics constraint
    //          to ensure no wheel velocity goes above max velocity
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    trajectoryConfig.setKinematics(DriveTrainConstants.kDriveKinematics);

    // Step 2. Create a "trajectory Generator" object
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

    // Step 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Step 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            subsystem::getPoseMeters,
            DriveTrainConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            subsystem::setSwerveModulesStates,
            subsystem);

    // Step 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
                new InstantCommand(() -> subsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> subsystem.stopSwerveMotors())
            );
    
    }
}