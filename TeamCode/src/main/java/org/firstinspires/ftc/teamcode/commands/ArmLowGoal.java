package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to drive the arm to the low basket position.
 */
public class ArmLowGoal extends CommandBase {

    private final Arm arm;
    private final GamePiece gamePiece;

    public ArmLowGoal(Arm a_arm, GamePiece g_gamePiece) {
        arm = a_arm;
        gamePiece = g_gamePiece;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double target = (gamePiece.is_sample()) ?
                ArmConstants.Sample.angle_mid :
                ArmConstants.Specimen.angle_mid ;
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
