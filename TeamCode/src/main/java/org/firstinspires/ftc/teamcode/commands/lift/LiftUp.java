package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenLift;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;

/**
 * A command to nudge the lift down.
 */
public class LiftUp extends CommandBase {

    private final SpecimenLift m_lift;
    private double target;

    public LiftUp(SpecimenLift lift) {
        m_lift = lift;
        addRequirements(m_lift);
    }

    @Override
    public void execute() {
        m_lift.runToPosition(
                m_lift.current_target +
                3 * Constants.LiftConstants.threshold);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
