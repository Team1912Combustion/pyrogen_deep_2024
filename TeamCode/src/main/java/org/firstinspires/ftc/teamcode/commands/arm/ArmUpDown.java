package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorFullIn;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorHighGoal;
import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to nudge the arm down.
 */
public class ArmUpDown extends SequentialCommandGroup {

    public ArmUpDown(Arm arm, Elevator elevator, GamePiece gamePiece) {
        addCommands(
                new ArmHighGoal(arm, gamePiece),
                new WaitCommand(200),
                new ElevatorHighGoal(elevator, gamePiece).withTimeout(5000),
                new WaitCommand(1000),
                new ElevatorFullIn(elevator).withTimeout(5000),
                new WaitCommand(200),
                new ArmIntake(arm, gamePiece)
        );
        addRequirements(arm);
    }
}
