package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the arm to the intake position.
 */
public class ArmIntake extends CommandBase {

    private final Arm m_arm;

    public ArmIntake(Arm arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.runToPosition(Constants.ArmConstants.intake);
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
