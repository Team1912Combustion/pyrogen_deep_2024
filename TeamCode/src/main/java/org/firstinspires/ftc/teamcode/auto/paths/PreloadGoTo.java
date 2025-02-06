package org.firstinspires.ftc.teamcode.auto.paths;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.auto.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.robot.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.robot.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.team1912.pyrogen.pyrolib.ftclib.command.InstantCommand;
import org.team1912.pyrogen.pyrolib.ftclib.command.MecanumControllerCommand;
import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.controller.PIDController;
import org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Translation2d;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.Trajectory;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.TrajectoryConfig;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.TrajectoryGenerator;

@TeleOp
public class PreloadGoTo extends SequentialCommandGroup {
    public PreloadGoTo(Drive drive, Odometry odometry) {

        Pose2d newPose = new Pose2d(24., 0., Rotation2d.fromDegrees(0.));
        Pose2d oldPose = odometry.getPose();
        double timeout = 15.;
        double error = 1.;

        GoToPoseCommand goToPoseCommand =
                new GoToPoseCommand(
                        newPose,
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
