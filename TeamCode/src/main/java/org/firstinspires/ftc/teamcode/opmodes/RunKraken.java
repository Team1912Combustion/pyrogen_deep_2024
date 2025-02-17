package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.arm.ArmClearBarrier;
import org.firstinspires.ftc.teamcode.commands.arm.ArmElevHighGoal;
import org.firstinspires.ftc.teamcode.commands.arm.ArmElevHome;
import org.firstinspires.ftc.teamcode.commands.arm.ArmElevIntake;
import org.firstinspires.ftc.teamcode.commands.arm.ArmElevLowGoal;
import org.firstinspires.ftc.teamcode.commands.arm.ArmDown;
import org.firstinspires.ftc.teamcode.commands.arm.ArmUp;
import org.firstinspires.ftc.teamcode.commands.claw.ClawToggle;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorDown;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorFullIn;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorUp;
import org.firstinspires.ftc.teamcode.commands.lift.LiftHighGoal;
import org.firstinspires.ftc.teamcode.commands.lift.LiftLowGoal;
import org.firstinspires.ftc.teamcode.commands.lift.LiftIntake;
import org.firstinspires.ftc.teamcode.commands.lift.LiftScore;
import org.firstinspires.ftc.teamcode.commands.specimen.SpecimenToggle;

import org.firstinspires.ftc.teamcode.commands.lift.LiftDown;
import org.firstinspires.ftc.teamcode.commands.lift.LiftUp;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandOpMode;
import org.team1912.pyrogen.pyrolib.ftclib.command.RunCommand;
import org.team1912.pyrogen.pyrolib.ftclib.command.InstantCommand;
import org.team1912.pyrogen.pyrolib.ftclib.gamepad.GamepadEx;
import org.team1912.pyrogen.pyrolib.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp
public class RunKraken extends CommandOpMode {

    public void initialize() {

        GamepadEx driverStick = new GamepadEx(gamepad1);
        GamepadEx opStick = new GamepadEx(gamepad2);

        GamePiece gamePiece = new GamePiece();

        Odometry odometry = new Odometry(hardwareMap, telemetry);

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

        SpecimenLift lift = new SpecimenLift(hardwareMap, telemetry);
        register(lift);

        Elevator elevator = new Elevator(hardwareMap, arm, telemetry);
        register(elevator);

        Claw claw = new Claw(hardwareMap, telemetry);
        register(claw);

        Specimen specimen = new Specimen(hardwareMap, telemetry);
        register(specimen);

        // button bindings for the mechanisms

        // adjustments for arm
        driverStick.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).
                whenPressed(new ArmClearBarrier(arm));
        driverStick.getGamepadButton(GamepadKeys.Button.DPAD_UP).
                whenPressed(new ArmDown(arm));
        driverStick.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).
                whenPressed(new ArmUp(arm));

        // adjustments for elevator
        driverStick.getGamepadButton(GamepadKeys.Button.Y).
                whenPressed(new ElevatorDown(elevator));
        driverStick.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(new ElevatorUp(elevator));
        driverStick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawToggle(claw));

        // sample scoring and floor intake
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_UP).
                  whenPressed(new ArmElevHighGoal(arm, elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).
                  whenPressed(new ArmElevLowGoal(arm,elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).
                  whenPressed(new ArmElevIntake(arm, elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).
                whenPressed(new ArmElevHome(arm, elevator));
        opStick.getGamepadButton(GamepadKeys.Button.BACK).
                whenPressed(new ElevatorFullIn(elevator));
        // claw
        opStick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawToggle(claw));


        // specimen lift
        opStick.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new LiftHighGoal(lift));
        opStick.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new LiftLowGoal(lift));
        opStick.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new LiftIntake(lift));
        opStick.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftScore(lift));
        // specimen grabber
        opStick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SpecimenToggle(specimen));
        // adjustments for lift
        opStick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).
                whenPressed(new LiftDown(lift));
        opStick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).
                whenPressed(new LiftUp(lift));

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
        run();
    }
}
