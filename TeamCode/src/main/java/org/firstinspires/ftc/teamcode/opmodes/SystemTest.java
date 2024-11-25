package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.pyrolib.OTOS.OTOSSensor;
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

@TeleOp
public class SystemTest extends CommandOpMode {

    private Drive m_drive;
    private Arm m_arm;
    private Elevator m_elevator;
    private Intake m_intake;
    private DefaultDrive m_driveCommand;
    private GamepadEx m_driverStick;
    private GamepadEx m_opStick;

    public void initialize() {

        m_driverStick = new GamepadEx(gamepad1);
        m_opStick = new GamepadEx(gamepad2);

        // get our OTOS sensor
        OTOSSensor m_OTOS = hardwareMap.get(OTOSSensor.class, "sensor_otos");

        // create our drive object
        m_drive = new Drive(hardwareMap, "front_left", "front_right",
                "back_left", "back_right", m_OTOS);
        register(m_drive);
        m_driveCommand = new DefaultDrive(m_drive,
                () -> m_driverStick.getLeftX(),
                () -> m_driverStick.getLeftY(),
                () -> m_driverStick.getRightX());
        m_drive.setDefaultCommand(m_driveCommand);
        m_arm = new Arm(hardwareMap, "arm");
        register(m_arm);
        m_elevator = new Elevator(hardwareMap, "elevator");
        register(m_elevator);
        m_intake = new Intake(hardwareMap, "intake");
        register(m_intake);

        // button bindings for the mechanisms
        m_opStick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenHeld(new InstantCommand(m_intake::runIn, m_intake))
            .whenReleased(new InstantCommand(m_intake::stop, m_intake));
        m_opStick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new InstantCommand(m_intake::runOut, m_intake))
                .whenReleased(new InstantCommand(m_intake::stop, m_intake));
        m_opStick.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new ArmHighBasket(m_arm));
        m_opStick.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ArmLowBasket(m_arm));
        m_opStick.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ArmIntake(m_arm));

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));

    }
}
