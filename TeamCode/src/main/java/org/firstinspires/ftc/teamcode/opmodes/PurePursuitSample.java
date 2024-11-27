package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.commands.PyroPPCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.waypoints.GeneralWaypoint;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Estimator;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class PurePursuitSample extends CommandOpMode {

    private Odometry m_robotOdometry;
    private Estimator m_estimator;
    private PyroPPCommand ppCommand;
    private Drive m_robotDrive;
    private Vision m_vision;

    @Override
    public void initialize() {
        m_vision = new Vision(hardwareMap, telemetry);
        m_robotOdometry = new Odometry(hardwareMap);
        m_estimator = new Estimator(m_robotOdometry, m_vision);

        // create our drive object
        m_robotDrive = new Drive(hardwareMap);

        // create our pure pursuit command
        ppCommand = new PyroPPCommand(
                m_robotDrive, m_estimator,
                new StartWaypoint(0, 0),
                new GeneralWaypoint(24, 0, 0.8, 0.8, 6),
                new EndWaypoint(
                        24, 24, degreesToRadians(90.), 0.5,
                        0.5, 6, 0.8, 1
                )
        );

        // schedule the command
        schedule(ppCommand);
    }

    double degreesToRadians(double deg) {
        return deg * Math.PI / 180.;
    }

}
