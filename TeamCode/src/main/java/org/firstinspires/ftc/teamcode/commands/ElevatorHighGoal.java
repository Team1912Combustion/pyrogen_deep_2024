package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

/**
 * A command to run the elevator to the high basket.
 */
public class ElevatorHighGoal extends CommandBase {

    private final Elevator elevator;
    private final GamePiece gamePiece;

    public ElevatorHighGoal(Elevator e_elevator, GamePiece g_gamePiece) {
        elevator = e_elevator;
        gamePiece = g_gamePiece;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        int position = (gamePiece.sample()) ?
                Constants.ElevatorConstants.Sample.high_goal :
                Constants.ElevatorConstants.Specimen.high_goal;
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
