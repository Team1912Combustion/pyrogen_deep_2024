package org.firstinspires.ftc.teamcode.commands;

import org.team1912.pyrogen.pyrolib.ftclib.command.InstantCommand;
import org.team1912.pyrogen.pyrolib.ftclib.command.ParallelCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to drive the arm to the high basket position.
 */
public class ScoreHighBasket extends SequentialCommandGroup {

    public ScoreHighBasket(Arm arm, Elevator elevator, Claw claw, GamePiece gamePiece) {
        addCommands(
                new ParallelCommandGroup(
                        new ArmHighGoal(arm, gamePiece).withTimeout(2000),
                        new ElevatorHighGoal(elevator, gamePiece).withTimeout(2000)
                ),
                new InstantCommand(claw::goOpen).andThen(new WaitCommand(500)),
                new ParallelCommandGroup(
                        new ArmIntake(arm, gamePiece).withTimeout(2000),
                        new ElevatorFullIn(elevator).withTimeout(2000)
                )
        );
        addRequirements(arm, elevator, claw);
    }
}
