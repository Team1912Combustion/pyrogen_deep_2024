package org.firstinspires.ftc.teamcode.commands;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ClawHold extends CommandBase {

    private final Claw claw;

    public ClawHold(Claw c_claw) {
        claw = c_claw;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.goHold();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
