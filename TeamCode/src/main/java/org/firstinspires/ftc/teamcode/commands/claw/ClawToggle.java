package org.firstinspires.ftc.teamcode.commands.claw;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ClawToggle extends CommandBase {

    private final Claw claw;

    public ClawToggle(Claw c_claw) {
        claw = c_claw;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.Toggle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
