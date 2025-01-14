package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commands.*;

import org.firstinspires.ftc.teamcode.subsystems.Estimator;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Specimen;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

@TeleOp
public class SystemTest extends CommandOpMode {

    public void initialize() {

        GamepadEx driverStick = new GamepadEx(gamepad1);
        GamepadEx opStick = new GamepadEx(gamepad2);

        GamePiece gamePiece = new GamePiece();

        Vision vision = new Vision(hardwareMap, telemetry);
        Odometry odometry = new Odometry(hardwareMap);
        Estimator estimator = new Estimator(odometry, vision);

        // create our drive object
        Drive drive = new Drive(hardwareMap, telemetry);
        register(drive);
        DefaultDrive driveCommand = new DefaultDrive(drive,
                driverStick::getLeftX,
                driverStick::getLeftY,
                driverStick::getRightX,
                estimator::getRotation);
        drive.setDefaultCommand(driveCommand);

        Arm arm = new Arm(hardwareMap, telemetry);
        register(arm);

        Elevator elevator = new Elevator(hardwareMap, arm, telemetry);
        register(elevator);

        Intake intake = new Intake(hardwareMap, telemetry);
        register(intake);

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

        /*
        // intake
        driverStick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenHeld(new InstantCommand(intake::runIn, intake))
            .whenReleased(new InstantCommand(intake::stop, intake));
        driverStick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new InstantCommand(intake::runOut, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));
         */

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
