package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenLift;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;

/**
 * A command to nudge the lift down.
 */
public class LiftScore extends CommandBase {

    private final SpecimenLift m_lift;
    private double target;

    public LiftScore(SpecimenLift lift) {
        m_lift = lift;
        addRequirements(m_lift);
    }

    @Override
    public void execute() {
        m_lift.runToPosition( m_lift.current_target - 100);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
