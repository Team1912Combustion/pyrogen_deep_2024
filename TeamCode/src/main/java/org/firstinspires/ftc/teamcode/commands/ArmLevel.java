package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmLevel extends CommandBase {

    private final Arm arm;

    public ArmLevel(Arm a_arm) {
        arm = a_arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.runToAngle(ArmConstants.angle_level);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return arm.atTarget();
    }
}
