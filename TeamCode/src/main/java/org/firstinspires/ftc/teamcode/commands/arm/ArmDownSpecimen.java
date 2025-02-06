package org.firstinspires.ftc.teamcode.commands.arm;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * A command to nudge the arm down.
 */
public class ArmDownSpecimen extends CommandBase {

    private final Arm m_arm;
    private double target;

    public ArmDownSpecimen(Arm arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.runToAngle(
                m_arm.current_target -
                12. * Constants.ArmConstants.angle_threshold);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
