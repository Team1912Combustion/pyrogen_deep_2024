package org.firstinspires.ftc.teamcode.auto.paths;

import org.firstinspires.ftc.teamcode.auto.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.robot.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.robot.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.team1912.pyrogen.pyrolib.ftclib.command.InstantCommand;
import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.controller.PIDController;
import org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;

public class PreloadGoTo extends SequentialCommandGroup {
    public PreloadGoTo(Drive drive, Odometry odometry) {

        Pose2d newPose = new Pose2d(12., 0., Rotation2d.fromDegrees(90.));
        Pose2d oldPose = Pose2d.kZero;
        odometry.update(oldPose);
        double runtime = 5.;
        double timeout = 5.;
        double error = 1.;

        GoToPoseCommand goToPoseCommand =
                new GoToPoseCommand(
                        newPose,
                        runtime,
                        timeout,
                        error,
                        odometry::getPose,
                        DriveConstants.kinematics,
                        new PIDController(AutoConstants.kPXController, 0, 0),
                        new PIDController(AutoConstants.kPYController, 0, 0),
                        new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0,
                                AutoConstants.kThetaControllerConstraints),
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        drive::driveWithSpeeds);

        addCommands(
                new InstantCommand(() -> odometry.update(oldPose)),
                goToPoseCommand,
                new InstantCommand(() -> drive.drive(0, 0, 0))
        );
    }
}
