package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.PreloadHigh;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

@Autonomous(name="RunPreloadLow", preselectTeleOp="RunKraken")
public class RunPreloadLow extends CommandOpMode {

    @Override
    public void initialize() {

        Arm arm = new Arm(hardwareMap, telemetry);
        register(arm);

        Elevator elevator = new Elevator(hardwareMap, arm, telemetry);
        register(elevator);

        Claw claw = new Claw(hardwareMap, telemetry);
        register(claw);

        GamePiece gamePiece = new GamePiece();

        schedule(new PreloadHigh(this, hardwareMap, telemetry,
                arm, elevator, claw, gamePiece));
    }
}
