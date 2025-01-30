package org.firstinspires.ftc.teamcode.commands.tape;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Tape;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;

/**
 * A command to nudge the tape down.
 */
public class TapeDown extends CommandBase {

    private final Tape m_tape;
    private double target;

    public TapeDown(Tape tape) {
        m_tape = tape;
        addRequirements(m_tape);
    }

    @Override
    public void execute() {
        m_tape.runToPosition(
                m_tape.current_target -
                3. * Constants.TapeConstants.threshold);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
