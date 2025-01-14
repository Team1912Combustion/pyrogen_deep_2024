package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmElevLowGoal extends SequentialCommandGroup {

    public ArmElevLowGoal(Arm arm, Elevator elevator, GamePiece gamePiece) {
        addCommands(
                new ElevatorLowGoal(elevator, gamePiece),
                new WaitCommand(500),
                new ArmLowGoal(arm, gamePiece)
        );
        addRequirements(arm, elevator);

    }

}
