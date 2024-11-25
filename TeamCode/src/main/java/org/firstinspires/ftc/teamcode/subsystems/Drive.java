package org.firstinspires.ftc.teamcode.subsystems;

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
    public final OTOSOdometry m_odometry;

    public Drive(MotorEx frontLeftMotor, MotorEx frontRightMotor,
                 MotorEx backLeftMotor, MotorEx backRightMotor, OTOSSensor otos,
                 Pose2d initialPose) {
        m_drive = new MecanumDrive(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor);
        m_odometry = new OTOSOdometry(otos::getPose2d, initialPose);
    }

    public Drive(HardwareMap hMap, String frontLeftMotorName, String frontRightMotorName,
                 String backLeftMotorName, String backRightMotorName, OTOSSensor otos) {
        this(
                new MotorEx(hMap, frontLeftMotorName),
                new MotorEx(hMap, frontRightMotorName),
                new MotorEx(hMap, backLeftMotorName),
                new MotorEx(hMap, backRightMotorName),
                otos,
                new Pose2d(0., 0., Rotation2d.fromDegrees(0.))
        );
    }

    public Drive(HardwareMap hMap, String frontLeftMotorName, String frontRightMotorName,
                 String backLeftMotorName, String backRightMotorName) {
        OTOSSensor m_OTOS = hMap.get(OTOSSensor.class, "sensor_otos");
        m_drive = new MecanumDrive(
                new MotorEx(hMap, frontLeftMotorName),
                new MotorEx(hMap, frontRightMotorName),
                new MotorEx(hMap, backLeftMotorName),
                new MotorEx(hMap, backRightMotorName)
        );
        m_odometry = new OTOSOdometry(m_OTOS::getPose2d,
                new Pose2d(0., 0., Rotation2d.fromDegrees(0.)));
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed,
                m_odometry.getPose().getRotation().getDegrees());
    }
}
