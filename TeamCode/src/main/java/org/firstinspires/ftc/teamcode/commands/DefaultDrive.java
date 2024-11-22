package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDrive extends CommandBase {

    private final Drive m_drive;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param strafe    The control input for strafing left/right
     * @param rotation  The control input for turning
     */
    public DefaultDrive(Drive subsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier rotation) {
        m_drive = subsystem;
        m_strafe = strafe;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(
                -1.*m_strafe.getAsDouble(),
                -1.*m_forward.getAsDouble(),
                -1.*m_rotation.getAsDouble()
        );
    }

}
