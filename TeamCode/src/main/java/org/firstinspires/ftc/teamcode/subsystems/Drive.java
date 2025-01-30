package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.team1912.pyrogen.pyrolib.OTOS.OTOSSensor;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.robot.Constants;

import org.team1912.pyrogen.pyrolib.ftclib.command.SubsystemBase;
import org.team1912.pyrogen.pyrolib.ftclib.drivebase.MecanumDrive;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.Motor;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants.DriveConstants;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive extends SubsystemBase {

    private final Telemetry telemetry;
    private final MecanumDrive m_drive;
    private boolean robotCentric;
    private boolean squareInputs;
    private static OTOSSensor imu;

    public final MecanumDriveKinematics kinematics;

    public Drive(MotorEx frontLeftMotor, MotorEx frontRightMotor,
                 MotorEx backLeftMotor, MotorEx backRightMotor, OTOSSensor i_imu,
                 Telemetry t_telemetry) {

        m_drive = new MecanumDrive(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor
        );
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        imu = i_imu;
        telemetry = t_telemetry;
        robotCentric = true;
        squareInputs = true;
        kinematics = new MecanumDriveKinematics(
                        DriveConstants.frontLeftInches,
                        DriveConstants.frontRightInches,
                        DriveConstants.backLeftInches,
                        DriveConstants.backRightInches);

    }

    public Drive(HardwareMap hMap, String frontLeftMotorName, String frontRightMotorName,
                 String backLeftMotorName, String backRightMotorName, OTOSSensor i_imu,
                 Telemetry t_telemetry) {
        this(
                new MotorEx(hMap, frontLeftMotorName),
                new MotorEx(hMap, frontRightMotorName),
                new MotorEx(hMap, backLeftMotorName),
                new MotorEx(hMap, backRightMotorName),
                i_imu, t_telemetry
        );
    }

    public Drive(HardwareMap hMap, Telemetry t_telemetry) {
        this(
                new MotorEx(hMap, Constants.DriveConstants.front_left_name),
                new MotorEx(hMap, Constants.DriveConstants.front_right_name),
                new MotorEx(hMap, Constants.DriveConstants.back_left_name),
                new MotorEx(hMap, Constants.DriveConstants.back_right_name),
                hMap.get(OTOSSensor.class, Constants.OdometryConstants.sensor_name),
                t_telemetry
        );
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0, squareInputs);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed,
                                  Rotation2d poseRot2d) {
        if (robotCentric) {
            m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed,
                    1.0, squareInputs);
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
