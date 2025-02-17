package org.firstinspires.ftc.teamcode.commands.elevator;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

/**
 * A command to run the elevator back in.
 */
public class ElevatorFullIn extends CommandBase {

    private final Elevator m_elevator;

    public ElevatorFullIn(Elevator elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.runToPosition(Constants.ElevatorConstants.full_in);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return m_elevator.atTarget();
    }
}
