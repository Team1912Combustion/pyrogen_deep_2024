package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.arm.ArmDownSpecimen;
import org.firstinspires.ftc.teamcode.commands.arm.ArmElevHighGoal;
import org.firstinspires.ftc.teamcode.commands.arm.ArmElevHome;
import org.firstinspires.ftc.teamcode.commands.arm.ArmElevIntake;
import org.firstinspires.ftc.teamcode.commands.arm.ArmElevLowGoal;
import org.firstinspires.ftc.teamcode.commands.claw.ClawToggle;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorFullIn;
import org.firstinspires.ftc.teamcode.commands.arm.LiftSpecimen;
import org.firstinspires.ftc.teamcode.commands.specimen.SpecimenToggle;

import org.firstinspires.ftc.teamcode.commands.lift.LiftDown;
import org.firstinspires.ftc.teamcode.commands.lift.LiftUp;
import org.firstinspires.ftc.teamcode.commands.Winch.WinchDown;
import org.firstinspires.ftc.teamcode.commands.Winch.WinchUp;

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

        Lift lift = new Lift(hardwareMap, telemetry);
        register(lift);

        Winch winch = new Winch(hardwareMap, telemetry);
        register(winch);

        Elevator elevator = new Elevator(hardwareMap, arm, telemetry);
        register(elevator);

        Claw claw = new Claw(hardwareMap, telemetry);
        register(claw);

        Specimen specimen = new Specimen(hardwareMap, telemetry);
        register(specimen);

        // button bindings for the mechanisms

        // arm
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_UP).
                  whenPressed(new ArmElevHighGoal(arm, elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).
                  whenPressed(new ArmElevLowGoal(arm,elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).
                  whenPressed(new ArmElevIntake(arm, elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).
                whenPressed(new ArmElevHome(arm, elevator));
        opStick.getGamepadButton(GamepadKeys.Button.BACK).
                whenPressed(new ArmDownSpecimen(arm));

        // elevator
        //opStick.getGamepadButton(GamepadKeys.Button.Y).
        //        whenPressed(new ElevatorHighGoal(elevator, gamePiece));
        //opStick.getGamepadButton(GamepadKeys.Button.X).
        //        whenPressed(new ElevatorLowGoal(elevator, gamePiece));
        opStick.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(new ElevatorFullIn(elevator));

        // adjustments for lift
        opStick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).
                  whenPressed(new LiftDown(lift));
        opStick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).
                  whenPressed(new LiftUp(lift));

        // tape adjustment
        driverStick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).
                whenPressed(new WinchDown(winch));
        driverStick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).
                whenPressed(new WinchUp(winch));

        // claw
        opStick.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ClawToggle(claw));
        opStick.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SpecimenToggle(specimen));
        opStick.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new LiftSpecimen(arm));

        opStick.getGamepadButton(GamepadKeys.Button.START).
                whenPressed(new InstantCommand(gamePiece::toggle));

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
        run();
    }
}
