package org.firstinspires.ftc.teamcode.oldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.ArmUpDown;
import org.firstinspires.ftc.teamcode.commands.PyroPPCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@Autonomous
public class ArmTest extends CommandOpMode {

    @Override
    public void initialize() {

        Arm arm = new Arm(hardwareMap, telemetry);
        Elevator elevator = new Elevator(hardwareMap, arm, telemetry);

        // schedule the command
        schedule(new ArmUpDown(arm, elevator));
    }
}
