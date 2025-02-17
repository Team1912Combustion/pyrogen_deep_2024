package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;

/**
 * A command to drive the arm to the high basket position.
 */
public class ArmClearBarrier extends CommandBase {

    private final Arm arm;

    public ArmClearBarrier(Arm a_arm) {
        arm = a_arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double target = Constants.ArmConstants.angle_barrier;
        arm.runToAngle(target);
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
