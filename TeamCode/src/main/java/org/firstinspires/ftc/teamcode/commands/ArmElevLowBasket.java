package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmElevLowBasket extends SequentialCommandGroup {

    public ArmElevLowBasket(Arm arm, Elevator elevator) {
        addCommands(
                new ElevatorFullIn(elevator),
                new WaitCommand(500),
                new ArmLowBasket(arm)
        );
        addRequirements(arm, elevator);

    }

}
