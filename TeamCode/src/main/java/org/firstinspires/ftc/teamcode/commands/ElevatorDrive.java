package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.function.DoubleSupplier;

/**
 * A command to run the elevator back in.
 */
public class ElevatorDrive extends CommandBase {

    private final Elevator m_elevator;
    private final DoubleSupplier m_speed;


    public ElevatorDrive(Elevator elevator, DoubleSupplier speed) {
        m_elevator = elevator;
        m_speed = speed;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.move(m_speed.getAsDouble());
    }


}
