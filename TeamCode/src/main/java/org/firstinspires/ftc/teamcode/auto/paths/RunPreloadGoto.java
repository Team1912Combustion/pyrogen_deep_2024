package org.firstinspires.ftc.teamcode.auto.paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandOpMode;

@Autonomous(name="RunPreloadGoto", preselectTeleOp="RunKraken")
public class RunPreloadGoto extends CommandOpMode {

    @Override
    public void initialize() {

        Odometry odometry = new Odometry(hardwareMap, telemetry);
        register(odometry);

        // create our drive object
        Drive drive = new Drive(hardwareMap, telemetry);
        register(drive);

        schedule(new PreloadGoTo(drive, odometry));
        run();

    }
}
