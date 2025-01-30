package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorLowGoal;
import org.team1912.pyrogen.pyrolib.ftclib.command.ParallelCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmElevLowGoal extends SequentialCommandGroup {

    public ArmElevLowGoal(Arm arm, Elevator elevator, GamePiece gamePiece) {
        addCommands(
                new ParallelCommandGroup(
                    new ElevatorLowGoal(elevator, gamePiece),
                    new ArmLowGoal(arm, gamePiece)
                )
        );
        addRequirements(arm, elevator);
    }
}
