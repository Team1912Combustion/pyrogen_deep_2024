package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.Preload;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;

@Autonomous(name="RunPreload", preselectTeleOp="RunKraken")
public class RunPreload extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new Preload(this, hardwareMap, telemetry));
    }
}
