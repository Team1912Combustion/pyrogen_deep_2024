package org.firstinspires.ftc.teamcode.auto.paths;

import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.robot.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.controller.PIDController;
import org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.command.InstantCommand;
import org.team1912.pyrogen.pyrolib.ftclib.command.MecanumControllerCommand;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Translation2d;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.Trajectory;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.TrajectoryConfig;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.TrajectoryGenerator;

public class PreloadTraj extends SequentialCommandGroup {
    public PreloadTraj(Drive drive, Odometry odometry) {

        TrajectoryConfig config =
                new TrajectoryConfig(
                        //AutoConstants.kMaxSpeedMetersPerSecond,
                        1.,
                        //AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        1.)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        Pose2d.kZero,
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(3, 0),
                                new Translation2d(6, 0),
                                new Translation2d(9, 0)),
                        new Pose2d(12., 0, Rotation2d.kZero),
                        config);

        MecanumControllerCommand mecanumControllerCommand =
                new MecanumControllerCommand(
                        exampleTrajectory,
                        odometry::getPose,
                        DriveConstants.kinematics,

                        // Position controllers
                        new PIDController(AutoConstants.kPXController, 0, 0),
                        new PIDController(AutoConstants.kPYController, 0, 0),
                        new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0,
                                AutoConstants.kThetaControllerConstraints),

                        // Needed for normalizing wheel speeds
                        AutoConstants.kMaxSpeedMetersPerSecond,

                        // Drivetrain Consumer for MecanumDriveWheelSpeeds
                        drive::driveWithSpeeds);

        addCommands(
                new InstantCommand(() -> odometry.update(exampleTrajectory.getInitialPose())),
                mecanumControllerCommand,
                new InstantCommand(() -> drive.drive(0, 0, 0))
        );
    }
}
