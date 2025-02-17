package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenLift;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;

/**
 * A command to run the elevator to the high basket.
 */
public class LiftIntake extends CommandBase {

    private final SpecimenLift lift;

    public LiftIntake(SpecimenLift l_lift) {
        lift = l_lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        int target = Constants.LiftConstants.intake;
        lift.runToPosition(target);
    }

    @Override
    public void end(boolean interrupted) {
        lift.stop();
    }

    @Override
    public boolean isFinished() {
        return lift.atTarget();
    }
}
