package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.auto.commands.PreloadParkSubStart;

@Autonomous(name="RunPreloadParkSub", preselectTeleOp="RunKraken")
public class RunPreloadParkSub extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new PreloadParkSubStart(this, hardwareMap, telemetry));
    }
}
