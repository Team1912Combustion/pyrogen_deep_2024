package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pyrolib.OTOS.OTOSSensor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.kinematics.OTOSOdometry;
import org.firstinspires.ftc.teamcode.robot.Constants.OdometryConstants;

public class Odometry extends SubsystemBase {

    public final OTOSOdometry m_otosOdometry;

    public Odometry(OTOSSensor otos, Pose2d initialPose) {
        otos.calibrateImu();
        m_otosOdometry = new OTOSOdometry(otos, initialPose);
    }

    public Odometry(OTOSSensor otos) {
        this( otos, new Pose2d(0., 0., Rotation2d.fromDegrees(0.)) );
    }

    public Odometry(HardwareMap hMap, Pose2d initialPose) {
        this( hMap.get(OTOSSensor.class, OdometryConstants.sensor_name), initialPose );
    }

    public Odometry(HardwareMap hMap) {
        this( hMap.get(OTOSSensor.class, OdometryConstants.sensor_name) );
    }

    public Pose2d getPose() {
        return m_otosOdometry.getPose();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void update() {
        m_otosOdometry.updatePose();
    }

    @Override
    public void periodic() { this.update(); }

}
