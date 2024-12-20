package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.robot.Constants;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants.DriveConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive extends SubsystemBase {

    private final MecanumDrive m_drive;
    private boolean robotCentric;
    private boolean squareInputs;

    public Drive(MotorEx frontLeftMotor, MotorEx frontRightMotor,
                 MotorEx backLeftMotor, MotorEx backRightMotor) {
        m_drive = new MecanumDrive(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor
        );
        robotCentric = false;
        squareInputs = true;
    }

    public Drive(HardwareMap hMap, String frontLeftMotorName, String frontRightMotorName,
                 String backLeftMotorName, String backRightMotorName) {
        this(
                new MotorEx(hMap, frontLeftMotorName),
                new MotorEx(hMap, frontRightMotorName),
                new MotorEx(hMap, backLeftMotorName),
                new MotorEx(hMap, backRightMotorName)
        );
    }

    public Drive(HardwareMap hMap) {
        this(
                new MotorEx(hMap, Constants.DriveConstants.front_left_name),
                new MotorEx(hMap, Constants.DriveConstants.front_right_name),
                new MotorEx(hMap, Constants.DriveConstants.back_left_name),
                new MotorEx(hMap, Constants.DriveConstants.back_right_name)
        );
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0, squareInputs);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed,
                                  Rotation2d poseRot2d) {
        if (robotCentric) {
            m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed,
                    0.0, squareInputs);
        } else {
            m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed,
                    poseRot2d.getDegrees(), squareInputs);
        }
    }

    public void stop() {
        m_drive.stop();
    }

    public void swapMode () {
        robotCentric = ! robotCentric;
    }

    public void set_squareInputs(boolean squareInputs) {
        this.squareInputs = squareInputs;
    }
}
