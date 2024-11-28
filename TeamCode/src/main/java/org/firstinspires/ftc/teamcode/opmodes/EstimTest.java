package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commands.*;

import org.firstinspires.ftc.teamcode.subsystems.Estimator;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp
public class EstimTest extends CommandOpMode {

    private Drive m_drive;
    private DefaultDrive m_driveCommand;
    private Vision m_vision;
    private Estimator m_estimator;
    private Odometry m_odometry;
    private GamepadEx m_driverStick;

    public void initialize() {

        m_driverStick = new GamepadEx(gamepad1);

        m_vision = new Vision(hardwareMap, telemetry);
        m_odometry = new Odometry(hardwareMap);
        m_estimator = new Estimator(m_odometry, m_vision);

        // create our drive object
        m_drive = new Drive(hardwareMap);
        register(m_drive);
        m_driveCommand = new DefaultDrive(m_drive,
                () -> m_driverStick.getLeftX(),
                () -> m_driverStick.getLeftY(),
                () -> m_driverStick.getRightX(),
                () -> m_estimator.getRotation());
        m_drive.setDefaultCommand(m_driveCommand);

        telemetry.addLine(String.format("Estimator XYZ %6.1f %6.1f %6.1f  (inch)",
                m_estimator.getPose().getX(),
                m_estimator.getPose().getY(),
                m_estimator.getPose().getHeading()));

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
        run();
    }
}
