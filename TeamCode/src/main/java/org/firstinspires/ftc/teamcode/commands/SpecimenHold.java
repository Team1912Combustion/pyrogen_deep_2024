package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Specimen;

/**
 * A command to drive the intake in.
 */
public class SpecimenHold extends CommandBase {

    private final Specimen m_specimen;

    public SpecimenHold(Specimen specimen) {
        m_specimen = specimen;
        addRequirements(m_specimen);
    }

    @Override
    public void execute() {
        m_specimen.goHold();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
