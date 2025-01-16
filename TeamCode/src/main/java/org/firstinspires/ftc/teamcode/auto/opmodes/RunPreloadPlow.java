package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.PreloadPlow;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;

@Autonomous(name="RunPreloadPlow", preselectTeleOp="RunKraken")
public class RunPreloadPlow extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new PreloadPlow(this, hardwareMap, telemetry));
    }
}
