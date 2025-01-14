package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * A command to nudge the arm up.
 */
public class ArmUp extends CommandBase {

    private final Arm m_arm;
    private double target;

    public ArmUp(Arm arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.runToAngle(
                m_arm.current_target +
                3.*Constants.ArmConstants.angle_threshold);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
