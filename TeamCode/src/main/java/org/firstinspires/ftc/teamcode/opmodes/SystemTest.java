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
public class SystemTest extends CommandOpMode {

    public void initialize() {

        GamepadEx m_driverStick = new GamepadEx(gamepad1);
        GamepadEx m_opStick = new GamepadEx(gamepad2);

        Vision m_vision = new Vision(hardwareMap, telemetry);
        Odometry m_odometry = new Odometry(hardwareMap);
        Estimator m_estimator = new Estimator(m_odometry, m_vision);

        // create our drive object
        Drive m_drive = new Drive(hardwareMap);
        register(m_drive);
        DefaultDrive m_driveCommand = new DefaultDrive(m_drive,
                m_driverStick::getLeftX,
                m_driverStick::getLeftY,
                m_driverStick::getRightX,
                m_estimator::getRotation);
        m_drive.setDefaultCommand(m_driveCommand);

        Elevator m_elevator = new Elevator(hardwareMap, "elevator", telemetry);
        register(m_elevator);

        Arm m_arm = new Arm(hardwareMap, "arm", m_elevator, telemetry);
        register(m_arm);

        Intake m_intake = new Intake(hardwareMap, "intake", telemetry);
        register(m_intake);

        // button bindings for the mechanisms

        // intake
        m_opStick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenHeld(new InstantCommand(m_intake::runIn, m_intake))
            .whenReleased(new InstantCommand(m_intake::stop, m_intake));
        m_opStick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new InstantCommand(m_intake::runOut, m_intake))
                .whenReleased(new InstantCommand(m_intake::stop, m_intake));

        // arm
        m_opStick.getGamepadButton(GamepadKeys.Button.DPAD_UP).
                  whenPressed(new ArmHighBasket(m_arm));
        m_opStick.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).
                  whenPressed(new ArmLowBasket(m_arm));
        m_opStick.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).
                  whenPressed(new ArmIntake(m_arm, m_elevator));

        // elevator
        m_opStick.getGamepadButton(GamepadKeys.Button.Y).
                whenPressed(new ElevatorHighBasket(m_elevator));
        m_opStick.getGamepadButton(GamepadKeys.Button.X).
                whenPressed(new ElevatorLowBasket(m_elevator));
        m_opStick.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(new ElevatorFullIn(m_elevator));

        m_opStick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).
                  whenPressed(new ArmDown(m_arm));
        m_opStick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).
                  whenPressed(new ArmUp(m_arm));

        /*
        m_opStick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).
                whileHeld(new IntakeOut(m_intake));
        m_opStick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).
                whileHeld(new IntakeIn(m_intake));
         */

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
        run();
    }
}
