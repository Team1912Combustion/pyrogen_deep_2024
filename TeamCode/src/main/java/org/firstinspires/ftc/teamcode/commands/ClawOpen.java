package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ClawOpen extends CommandBase {

    private final Claw claw;

    public ClawOpen(Claw c_claw) {
        claw = c_claw;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.goSafe();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
