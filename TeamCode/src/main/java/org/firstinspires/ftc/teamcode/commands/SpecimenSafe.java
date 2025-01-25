package org.firstinspires.ftc.teamcode.commands;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Specimen;

/**
 * A command to drive the intake in.
 */
public class SpecimenSafe extends CommandBase {

    private final Specimen m_specimen;

    public SpecimenSafe(Specimen specimen) {
        m_specimen = specimen;
        addRequirements(m_specimen);
    }

    @Override
    public void execute() {
        m_specimen.goSafe();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
