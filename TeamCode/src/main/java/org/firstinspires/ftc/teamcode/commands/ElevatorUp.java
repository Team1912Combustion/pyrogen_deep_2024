package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

/**
 * A command to run the elevator back in.
 */
public class ElevatorUp extends CommandBase {

    private final Elevator m_elevator;

    public ElevatorUp(Elevator elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.runToPosition(
                m_elevator.get_position() +
                Constants.ElevatorConstants.threshold);
    }
    @Override
    public boolean isFinished() {
        return true;
    }

}
