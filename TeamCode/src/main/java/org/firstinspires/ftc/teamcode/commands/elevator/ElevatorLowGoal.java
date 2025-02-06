package org.firstinspires.ftc.teamcode.commands.elevator;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to run the elevator to the high basket.
 */
public class ElevatorLowGoal extends CommandBase {

    private final Elevator elevator;
    private final GamePiece gamePiece;

    public ElevatorLowGoal(Elevator e_elevator, GamePiece g_gamePiece) {
        elevator = e_elevator;
        gamePiece = g_gamePiece;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        int arm_target = (gamePiece.is_sample()) ?
                Constants.ArmConstants.Sample.pos_mid :
                Constants.ArmConstants.Specimen.pos_mid;
        int target = (gamePiece.is_sample()) ?
                Constants.ElevatorConstants.Sample.low_goal :
                Constants.ElevatorConstants.Specimen.low_goal;
        int position = elevator.safeLimit(target, arm_target);
        elevator.runToPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return elevator.atTarget();
    }
}
