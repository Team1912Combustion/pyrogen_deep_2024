package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.PreloadParkObs;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;

@Autonomous(name="RunPreloadParkObs", preselectTeleOp="RunKraken")
public class RunPreloadParkObs extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new PreloadParkObs(this, hardwareMap, telemetry));
    }
}
