package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the arm to the intake position.
 */
public class ArmIntake extends CommandBase {

    private final Arm m_arm;
    private final Elevator m_elevator;

    public ArmIntake(Arm arm, Elevator elevator) {
        m_arm = arm;
        m_elevator = elevator;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        double new_angle = ( 1. - .9 * elevatorFraction() ) *
                           Constants.ArmConstants.angle_intake;
        m_arm.runToAngle(new_angle);
    }

    public double elevatorFraction() {
        return (double) m_elevator.get_position() /
               (double) Constants.ElevatorConstants.full_out;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stop();
    }

    @Override
    public boolean isFinished() {
        return m_arm.atTarget();
    }
}
