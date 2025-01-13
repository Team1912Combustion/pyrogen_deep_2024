package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.auto.commands.PreloadHigh;

@Autonomous
public class RunPreloadHigh extends CommandOpMode {

    @Override
    public void initialize() {

        Arm m_arm = new Arm(hardwareMap, telemetry);
        register(m_arm);

        Elevator m_elevator = new Elevator(hardwareMap, m_arm, telemetry);
        register(m_elevator);

        Intake m_intake = new Intake(hardwareMap, telemetry);
        register(m_intake);

        schedule(new PreloadHigh(this, hardwareMap, telemetry,
                m_arm, m_elevator, m_intake));
    }
}
