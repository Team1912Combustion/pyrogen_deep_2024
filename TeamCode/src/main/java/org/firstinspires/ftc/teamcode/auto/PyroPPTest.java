package org.firstinspires.ftc.teamcode.auto;

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
public class PyroPPTest extends CommandOpMode {

    @Override
    public void initialize() {
        Vision m_vision = new Vision(hardwareMap, telemetry);
        Odometry m_robotOdometry = new Odometry(hardwareMap);
        Estimator m_estimator = new Estimator(m_robotOdometry, m_vision);
        Drive m_robotDrive = new Drive(hardwareMap);

        PyroPPCommand ppCommand = new PyroPPCommand(
                m_robotDrive, m_estimator,
                new StartWaypoint(0, 0),
                new GeneralWaypoint(24, 0, 0.8, 0.8, 6),
                new EndWaypoint(
                        24, 24, Math.toRadians(90.), 0.5,
                        0.5, 6, 0.8, 1
                )
        );

        // schedule the command
        schedule(ppCommand);
    }

}
