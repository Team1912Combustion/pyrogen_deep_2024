package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.auto.commands.PreloadPlowSide;

@Autonomous(name="RunPreloadPlowSide", preselectTeleOp="RunKraken")
@Disabled
public class RunPreloadPlowSide extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new PreloadPlowSide(this, hardwareMap, telemetry));
    }
}
