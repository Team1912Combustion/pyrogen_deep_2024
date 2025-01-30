package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorIntake;
import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmElevIntake extends SequentialCommandGroup {

    public ArmElevIntake(Arm arm, Elevator elevator, GamePiece gamePiece) {
        addCommands(
                new ElevatorIntake(elevator, gamePiece),
                new WaitCommand(500),
                new ArmIntake(arm, gamePiece)
        );
        addRequirements(arm, elevator);

    }

}
