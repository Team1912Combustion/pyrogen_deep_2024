package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;

/**
 * A command to nudge the lift down.
 */
public class LiftDown extends CommandBase {

    private final Lift m_lift;
    private double target;

    public LiftDown(Lift lift) {
        m_lift = lift;
        addRequirements(m_lift);
    }

    @Override
    public void execute() {
        m_lift.runToPosition(
                m_lift.current_target -
                3. * Constants.LiftConstants.threshold);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
