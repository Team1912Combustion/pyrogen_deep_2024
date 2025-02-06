package org.firstinspires.ftc.teamcode.commands.specimen;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Specimen;

/**
 * A command to drive the intake in.
 */
public class SpecimenToggle extends CommandBase {

    private final Specimen m_specimen;

    public SpecimenToggle(Specimen specimen) {
        m_specimen = specimen;
        addRequirements(m_specimen);
    }

    @Override
    public void execute() {
        m_specimen.Toggle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
