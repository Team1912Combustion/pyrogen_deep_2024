package org.firstinspires.ftc.teamcode.auto.paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

@Autonomous(name="RunPreloadTraj", preselectTeleOp="RunKraken")
public class RunPreload extends CommandOpMode {

    @Override
    public void initialize() {

        Odometry odometry = new Odometry(hardwareMap, telemetry);

        // create our drive object
        Drive drive = new Drive(hardwareMap, telemetry);
        register(drive);

        schedule(new PreloadTraj(drive, odometry));
    }
}
