package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmElevHighGoal extends SequentialCommandGroup {

    public ArmElevHighGoal(Arm arm, Elevator elevator, GamePiece gamePiece) {
        addCommands(
                new ParallelCommandGroup(
                    new ArmHighGoal(arm, gamePiece),
                    new ElevatorHighGoal(elevator, gamePiece)
                )
        );
        addRequirements(arm, elevator);

    }

}
