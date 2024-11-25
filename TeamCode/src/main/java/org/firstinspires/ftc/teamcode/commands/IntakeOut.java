package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * A command to drive the intake in.
 */
public class IntakeOut extends CommandBase {

    private final Intake m_intake;

    public IntakeOut(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.runOut();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
