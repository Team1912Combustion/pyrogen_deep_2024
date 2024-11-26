package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.Estimator;

import org.firstinspires.ftc.teamcode.pyrolib.OTOS.OTOSSensor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.kinematics.OTOSOdometry;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive extends SubsystemBase {

    private final MecanumDrive m_drive;
    private final Estimator m_estimator;

    public Drive(MotorEx frontLeftMotor, MotorEx frontRightMotor,
                 MotorEx backLeftMotor, MotorEx backRightMotor, Estimator estimator) {
        m_estimator = estimator;
        m_drive = new MecanumDrive(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor
        );
    }

    public Drive(HardwareMap hMap, String frontLeftMotorName, String frontRightMotorName,
                 String backLeftMotorName, String backRightMotorName, Estimator estimator) {
        this(
                new MotorEx(hMap, frontLeftMotorName),
                new MotorEx(hMap, frontRightMotorName),
                new MotorEx(hMap, backLeftMotorName),
                new MotorEx(hMap, backRightMotorName),
                estimator
        );
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed,
                m_estimator.getPose().getRotation().getDegrees());
    }
}
