package org.firstinspires.ftc.teamcode.commands;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to drive the arm to the high basket position.
 */
public class ArmHighGoal extends CommandBase {

    private final Arm arm;
    private final GamePiece gamePiece;

    public ArmHighGoal(Arm a_arm, GamePiece g_gamePiece) {
        arm = a_arm;
        gamePiece = g_gamePiece;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double target = (gamePiece.is_sample()) ?
                Constants.ArmConstants.Sample.angle_high :
                Constants.ArmConstants.Specimen.angle_high ;
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
