package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.function.DoubleSupplier;

/**
 * A command to run the elevator to the low basket.
 */
public class ElevatorLowBasket extends CommandBase {

    private final Elevator m_elevator;

    public ElevatorLowBasket(Elevator elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.runToPosition(Constants.ElevatorConstants.low_basket);
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
