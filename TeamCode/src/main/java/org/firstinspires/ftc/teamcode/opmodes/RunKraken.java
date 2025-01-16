package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;

@TeleOp
public class RunKraken extends CommandOpMode {

    public void initialize() {

        GamepadEx driverStick = new GamepadEx(gamepad1);
        GamepadEx opStick = new GamepadEx(gamepad2);

        GamePiece gamePiece = new GamePiece();

        Odometry odometry = new Odometry(hardwareMap);

        // create our drive object
        Drive drive = new Drive(hardwareMap, telemetry);
        register(drive);
        DefaultDrive driveCommand = new DefaultDrive(drive,
                driverStick::getLeftX,
                driverStick::getLeftY,
                driverStick::getRightX,
                odometry::getRotation);
        drive.setDefaultCommand(driveCommand);

        Arm arm = new Arm(hardwareMap, telemetry);
        register(arm);

        Elevator elevator = new Elevator(hardwareMap, arm, telemetry);
        register(elevator);

        Claw claw = new Claw(hardwareMap, telemetry);
        register(claw);

        Specimen specimen = new Specimen(hardwareMap, telemetry);
        register(specimen);

        // button bindings for the mechanisms

        // claw
        driverStick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawOpen(claw))
                .whenReleased(new ClawHold(claw));
        driverStick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawOpen(claw))
                .whenReleased(new ClawHold(claw));

        // arm
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_UP).
                  whenPressed(new ArmHighGoal(arm, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).
                  whenPressed(new ArmElevLowGoal(arm,elevator,gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).
                  whenPressed(new ArmElevIntake(arm, elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).
                whenPressed(new ArmDownSpecimen(arm));

        // elevator
        opStick.getGamepadButton(GamepadKeys.Button.Y).
                whenPressed(new ElevatorHighGoal(elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.X).
                whenPressed(new ElevatorLowGoal(elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(new ElevatorFullIn(elevator));

        // adjustments for arm
        opStick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).
                  whenPressed(new ArmDown(arm));
        opStick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).
                  whenPressed(new ArmUp(arm));
        opStick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).
                whenPressed(new ArmDown(arm));
        opStick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).
                whenPressed(new ArmUp(arm));
        driverStick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).
                whenPressed(new ElevatorDown(elevator));
        driverStick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).
                whenPressed(new ElevatorUp(elevator));

        opStick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).
                whenPressed(new SpecimenHold(specimen));
        opStick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).
                whenPressed(new SpecimenSafe(specimen));

        opStick.getGamepadButton(GamepadKeys.Button.BACK).
                whenPressed(new InstantCommand(gamePiece::toggle));

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
        run();
    }
}
