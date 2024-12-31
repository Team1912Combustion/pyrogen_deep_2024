package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

/**
 * A command to run the elevator back in.
 */
public class ElevatorHang extends CommandBase {

    private final Elevator m_elevator;

    public ElevatorHang(Elevator elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.runToPosition(Constants.ElevatorConstants.hang);
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
