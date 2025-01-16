package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.AlliancePlowPark;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;

@Autonomous(name="RunAlliancePlowPark", preselectTeleOp="RunKraken")
public class RunAlliancePlowPark extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new AlliancePlowPark(this, hardwareMap, telemetry));
    }
}
