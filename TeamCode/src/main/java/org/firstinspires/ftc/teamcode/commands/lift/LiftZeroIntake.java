package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.commands.arm.ArmHighGoal;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorHighGoal;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenLift;
import org.team1912.pyrogen.pyrolib.ftclib.command.ParallelCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;

/**
 * A command to drive the arm to the low basket position.
 */
public class LiftZeroIntake extends SequentialCommandGroup {

    public LiftZeroIntake(SpecimenLift lift) {
        addCommands(
                new LiftZero(lift).withTimeout(1000),
                new LiftIntake(lift)
        );
        addRequirements(lift);
    }
}
