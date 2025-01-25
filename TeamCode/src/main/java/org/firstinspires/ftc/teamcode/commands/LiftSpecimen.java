package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * A command to nudge the arm up.
 */
public class LiftSpecimen extends CommandBase {

    private final Arm m_arm;
    private double target;

    public LiftSpecimen(Arm arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        double target = Constants.ArmConstants.Specimen.angle_mid ;
        m_arm.runToAngle(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
