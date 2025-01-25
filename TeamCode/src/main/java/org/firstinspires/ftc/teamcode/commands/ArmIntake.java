package org.firstinspires.ftc.teamcode.commands;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the arm to the intake position.
 */
public class ArmIntake extends CommandBase {

    private final Arm arm;
    private final GamePiece gamePiece;

    public ArmIntake(Arm a_arm, GamePiece g_gamePiece) {
        arm = a_arm;
        gamePiece =g_gamePiece;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double target = (gamePiece.is_sample()) ?
                ArmConstants.Sample.angle_intake :
                ArmConstants.Specimen.angle_intake ;
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
