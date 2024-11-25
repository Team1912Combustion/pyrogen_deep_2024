package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * A command to drive the arm to the high basket position.
 */
public class ScoreHighBasket extends SequentialCommandGroup {

    public ScoreHighBasket(Arm arm, Elevator elevator, Intake intake) {
        addCommands(
                new ParallelCommandGroup(new ArmHighBasket(arm), new ElevatorHighBasket(elevator)),
                new InstantCommand(intake::runIn),
                new WaitCommand(2000),
                new InstantCommand(intake::stop),
                new ParallelCommandGroup(new ArmIntake(arm), new ElevatorFullIn(elevator))
        );
        addRequirements(arm, elevator, intake);
    }
}
