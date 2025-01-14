package org.firstinspires.ftc.teamcode.oldAuto;

import org.firstinspires.ftc.teamcode.commands.PyroPPCommand;
import org.firstinspires.ftc.teamcode.pyrolib.OTOS.OTOSSensor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.Path;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.waypoints.GeneralWaypoint;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.purepursuit.waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Estimator;
import org.firstinspires.ftc.teamcode.subsystems.Screen;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class PyroPPTest extends CommandOpMode {

    @Override
    public void initialize() {
        OTOSSensor m_otos = hardwareMap.get(OTOSSensor.class, Constants.OdometryConstants.sensor_name);

        Vision m_vision = new Vision(hardwareMap, telemetry);
        Odometry m_robotOdometry = new Odometry(hardwareMap);
        Estimator m_estimator = new Estimator(m_robotOdometry, m_vision);
        Drive m_robotDrive = new Drive(hardwareMap, telemetry);
        Screen m_screen = new Screen(m_otos, m_robotOdometry, m_estimator, telemetry);

        m_estimator.resetPose(new Pose2d(0.,0., Rotation2d.fromDegrees(0.)));
        Path m_path = new Path();
        m_path.add(new StartWaypoint(m_estimator.getPose()));
        m_path.add(new GeneralWaypoint(20, 0, .8, 0.8, 6));
        m_path.add(new EndWaypoint( 20, -20, Math.toRadians(-90.), .8,
                0.8, 6, 0.8, 1 ) );
        m_path.setPathTimeout(15000); // path timeout in milliSeconds
        m_path.init();

        PyroPPCommand ppCommand = new PyroPPCommand( m_robotDrive, m_estimator, m_path, telemetry);

        ppCommand.initialize();

        // schedule the command
        schedule(ppCommand);
    }
}
