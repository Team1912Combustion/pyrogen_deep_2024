package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.Path;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.Waypoint;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Estimator;

/**
 * @author Jackson
 * @see Path
 */
public class PyroPPCommand extends CommandBase {

    private Drive m_drive;
    private Estimator m_odometry;
    private Path m_path;

    public PyroPPCommand(Drive drive, Estimator odometry, Waypoint... waypoints) {
        m_path = new Path(waypoints);
        m_drive = drive;
        m_odometry = odometry;
    }

    @Override
    public void initialize() {
        m_path.init();
    }

    public void addWaypoint(Waypoint waypoint) {
        m_path.add(waypoint);
    }

    public void addWaypoints(Waypoint... waypoints) {
        for (Waypoint waypoint : waypoints) this.addWaypoint(waypoint);
    }

    public void removeWaypointAtIndex(int index) {
        m_path.remove(index);
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        Pose2d robotPose = m_odometry.getPose();
        double[] motorSpeeds = m_path.loop(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), robotPose.getHeading());
        m_drive.drive(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2]);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return m_path.isFinished();
    }

}
