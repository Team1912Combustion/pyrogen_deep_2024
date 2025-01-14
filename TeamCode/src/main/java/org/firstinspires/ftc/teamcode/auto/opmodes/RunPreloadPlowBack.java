package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.PreloadPlowBack;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;

@Autonomous
public class RunPreloadPlowBack extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new PreloadPlowBack(this, hardwareMap, telemetry));
    }
}
