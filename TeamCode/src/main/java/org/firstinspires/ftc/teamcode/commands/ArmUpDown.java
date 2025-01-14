package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

/**
 * A command to nudge the arm down.
 */
public class ArmUpDown extends SequentialCommandGroup {

    public ArmUpDown(Arm arm, Elevator elevator) {
        addCommands(
                new ArmHighBasket(arm),
                new WaitCommand(200),
                new ElevatorHighBasket(elevator).withTimeout(5000),
                new WaitCommand(1000),
                new ElevatorFullIn(elevator).withTimeout(5000),
                new WaitCommand(200),
                new ArmIntake(arm, elevator)
        );
        addRequirements(arm);
    }
}
