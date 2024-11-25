package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmLowBasket extends CommandBase {

    private final Arm m_arm;

    public ArmLowBasket(Arm arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.runToPosition(Constants.ArmConstants.low_basket);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
