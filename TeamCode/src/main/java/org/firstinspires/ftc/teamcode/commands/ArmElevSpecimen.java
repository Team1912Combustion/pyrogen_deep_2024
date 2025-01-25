package org.firstinspires.ftc.teamcode.commands;

import org.team1912.pyrogen.pyrolib.ftclib.command.SequentialCommandGroup;
import org.team1912.pyrogen.pyrolib.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmElevSpecimen extends SequentialCommandGroup {

    public ArmElevSpecimen(Arm arm) {
        addCommands(
                new ArmDown(arm),
                new ArmDown(arm),
                new ArmUp(arm),
                new ArmUp(arm)
        );
        addRequirements(arm);

    }

}
