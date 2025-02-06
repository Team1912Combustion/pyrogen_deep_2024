package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.team1912.pyrogen.pyrolib.OTOS.OTOSSensor;
import org.team1912.pyrogen.pyrolib.ftclib.command.SubsystemBase;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.OTOSOdometry;
import org.firstinspires.ftc.teamcode.robot.Constants.OdometryConstants;

public class Odometry extends SubsystemBase {

    public final OTOSOdometry m_otosOdometry;
    private final Telemetry telemetry;

    public Odometry(OTOSSensor otos, Pose2d initialPose, Telemetry t_telemetry) {
        otos.calibrateImu();
        telemetry = t_telemetry;
        m_otosOdometry = new OTOSOdometry(otos, initialPose);
    }

    public Odometry(OTOSSensor otos, Telemetry telemetry) {
        this( otos, new Pose2d(0., 0., Rotation2d.fromDegrees(0.)), telemetry );
    }

    public Odometry(HardwareMap hMap, Pose2d initialPose, Telemetry telemetry) {
        this( hMap.get(OTOSSensor.class, OdometryConstants.sensor_name), initialPose, telemetry );
    }

    public Odometry(HardwareMap hMap, Telemetry telemetry) {
        this( hMap.get(OTOSSensor.class, OdometryConstants.sensor_name), telemetry );
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

    public void update(Pose2d newpose) {
        m_otosOdometry.updatePose(newpose);
    }

    @Override
    public void periodic() {
        telemetry.addLine(String.format("Odo Pose %f %f %f \n",
                getPose().getX(),
                getPose().getY(),
                getPose().getHeading()));
        //telemetry.update();
        this.update();
    }

}
