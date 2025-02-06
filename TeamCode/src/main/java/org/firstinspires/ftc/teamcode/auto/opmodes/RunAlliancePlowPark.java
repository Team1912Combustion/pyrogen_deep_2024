package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.AlliancePlowPark;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandOpMode;

@Autonomous(name="Right-PlowAlliance", preselectTeleOp="RunKraken")
public class RunAlliancePlowPark extends CommandOpMode {

    @Override
    public void initialize() {

        // schedule the command
        schedule(new AlliancePlowPark(this, hardwareMap, telemetry));
    }
}
