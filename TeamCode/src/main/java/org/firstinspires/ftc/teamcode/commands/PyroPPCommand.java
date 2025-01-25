package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.purepursuit.Path;
import org.team1912.pyrogen.pyrolib.ftclib.purepursuit.PathMotionProfile;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

public class PyroPPCommand extends CommandBase {

    private Drive m_drive;
    private Odometry m_odometry;
    private Path m_path;
    private Telemetry m_telemetry;

    public PyroPPCommand(Drive drive, Odometry odometry, Path path, Telemetry telemetry) {
        m_path = path;
        m_drive = drive;
        m_odometry = odometry;
        m_telemetry = telemetry;
        m_path.setMotionProfile(PyroMotionProfile());
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_odometry.getPose();
        double[] motorSpeeds = m_path.loop(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        m_drive.drive(motorSpeeds[1], -1.*motorSpeeds[0], -1.* motorSpeeds[2]);
        m_telemetry.addLine(String.format("pp pose %7.3f %7.3f %7.3f",
                robotPose.getX(),
                robotPose.getY(),
                robotPose.getHeading()));
        m_telemetry.addLine(String.format("pp speeds %7.3f %7.3f %7.3f",
                motorSpeeds[0],
                motorSpeeds[1],
                motorSpeeds[2]));
        m_telemetry.update();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return m_path.isFinished() || m_path.timedOut();
    }

    /**
     * Generates a Pyro PathMotionProfile.
     *
     * @return the Pyro PathMotionProfile.
     */
    private PathMotionProfile PyroMotionProfile() {
        // Create a default motion profile.
        // The default profile is a messy trapezoid(ish) curve.
        return new PathMotionProfile() {
            @Override
            public void decelerate(double[] motorSpeeds, double distanceToTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
                if (distanceToTarget < 6.) {
                    motorSpeeds[0] *= configuredMovementSpeed * ((distanceToTarget * .15) + 0.15);
                    motorSpeeds[1] *= configuredMovementSpeed * ((distanceToTarget * .15) + 0.15);
                    motorSpeeds[2] *= configuredTurnSpeed;
                } else {
                    motorSpeeds[0] *= configuredMovementSpeed;
                    motorSpeeds[1] *= configuredMovementSpeed;
                    motorSpeeds[2] *= configuredTurnSpeed;
                }
            }

            @Override
            public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
                if (distanceFromTarget < 6.) {
                    motorSpeeds[0] *= configuredMovementSpeed * ((distanceFromTarget * 0.15) + 0.15);
                    motorSpeeds[1] *= configuredMovementSpeed * ((distanceFromTarget * 0.15) + 0.15);
                    motorSpeeds[2] *= configuredTurnSpeed;
                } else {
                    motorSpeeds[0] *= configuredMovementSpeed;
                    motorSpeeds[1] *= configuredMovementSpeed;
                    motorSpeeds[2] *= configuredTurnSpeed;
                }
            }
        };
    }
}
