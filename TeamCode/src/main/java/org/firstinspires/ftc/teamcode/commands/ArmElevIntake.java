package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmElevIntake extends SequentialCommandGroup {

    public ArmElevIntake(Arm arm, Elevator elevator) {
        addCommands(
                new ElevatorFullIn(elevator),
                new WaitCommand(500),
                new ArmIntake(arm,elevator)
        );
        addRequirements(arm, elevator);

    }

}
