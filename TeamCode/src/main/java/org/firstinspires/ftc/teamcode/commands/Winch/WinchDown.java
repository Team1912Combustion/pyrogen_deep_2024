package org.firstinspires.ftc.teamcode.commands.Winch;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Winch;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;

/**
 * A command to nudge the tape down.
 */
public class WinchDown extends CommandBase {

    private final Winch m_winch;
    private double target;

    public WinchDown(Winch winch) {
        m_winch = winch;
        addRequirements(m_winch);
    }

    @Override
    public void execute() {
        m_winch.runToPosition(
                m_winch.current_target -
                3. * Constants.WinchConstants.threshold);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
