package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.auto.commands.PreloadSub;

@Autonomous
public class RunPreloadSub extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new PreloadSub(this, hardwareMap, telemetry));
    }
}
