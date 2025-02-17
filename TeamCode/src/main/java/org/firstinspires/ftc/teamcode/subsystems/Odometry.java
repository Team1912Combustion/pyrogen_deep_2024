package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.team1912.pyrogen.pyrolib.GoBildaPinpoint.GoBildaPinpointDriver;
import org.team1912.pyrogen.pyrolib.GoBildaPinpoint.PinpointSensor;
import org.team1912.pyrogen.pyrolib.ftclib.command.SubsystemBase;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.PinpointOdometry;
import org.firstinspires.ftc.teamcode.robot.Constants.OdometryConstants;

public class Odometry extends SubsystemBase {

    public final PinpointOdometry m_pinpointOdometry;
    private final Telemetry telemetry;

    public Odometry(PinpointSensor pinpoint, Pose2d initialPose, Telemetry t_telemetry) {
        pinpoint.resetPosAndIMU();
        telemetry = t_telemetry;
        m_pinpointOdometry = new PinpointOdometry(pinpoint, initialPose,
                OdometryConstants.xOffset, OdometryConstants.yOffset,
                OdometryConstants.xDirection, OdometryConstants.yDirection,
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    public Odometry(PinpointSensor pinpoint, Telemetry telemetry) {
        this( pinpoint, new Pose2d(0., 0., Rotation2d.fromDegrees(0.)), telemetry );
    }

    public Odometry(HardwareMap hMap, Pose2d initialPose, Telemetry telemetry) {
        this( hMap.get(PinpointSensor.class, OdometryConstants.sensor_name), initialPose, telemetry );
    }

    public Odometry(HardwareMap hMap, Telemetry telemetry) {
        this( hMap.get(PinpointSensor.class, OdometryConstants.sensor_name), telemetry );
    }

    public Pose2d getPose() {
        return m_pinpointOdometry.getPose();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void update() {
        m_pinpointOdometry.updatePose();
    }

    public void update(Pose2d newpose) {
        m_pinpointOdometry.updatePose(newpose);
    }

    @Override
    public void periodic() {
        telemetry.addLine(String.format("Odo Pose %f %f %f",
                getPose().getX(),
                getPose().getY(),
                getPose().getHeading()));
        //telemetry.update();
        this.update();
    }

}
