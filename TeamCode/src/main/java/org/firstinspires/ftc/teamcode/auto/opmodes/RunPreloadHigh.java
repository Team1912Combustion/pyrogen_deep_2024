package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.auto.commands.PreloadHigh;

@Autonomous
public class RunPreloadHigh extends CommandOpMode {

    @Override
    public void initialize() {

        Arm arm = new Arm(hardwareMap, telemetry);
        register(arm);

        Elevator elevator = new Elevator(hardwareMap, arm, telemetry);
        register(elevator);

        Intake intake = new Intake(hardwareMap, telemetry);
        register(intake);

        GamePiece gamePiece = new GamePiece();

        schedule(new PreloadHigh(this, hardwareMap, telemetry,
                arm, elevator, intake, gamePiece));
    }
}
