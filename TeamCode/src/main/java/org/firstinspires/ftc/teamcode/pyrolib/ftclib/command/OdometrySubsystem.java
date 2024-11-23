package org.firstinspires.ftc.teamcode.pyrolib.ftclib.command;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pyrolib.OTOS.OTOSSensor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.kinematics.OTOSOdometry;

public class OdometrySubsystem extends SubsystemBase {

    protected OTOSOdometry m_odometry;

    public OdometrySubsystem(HardwareMap hwmap, Pose2d initialPose) {
        // create our odometry object and subsystem
        OTOSSensor m_OTOS = hwmap.get(OTOSSensor.class, "sensor_otos");
        OTOSOdometry m_odometry = new OTOSOdometry(m_OTOS::getPose2d, initialPose);
    }

    public OdometrySubsystem(HardwareMap hwmap) {
        // create our odometry object and subsystem
        OTOSSensor m_OTOS = hwmap.get(OTOSSensor.class, "sensor_otos");
        Pose2d initialPose = new Pose2d(0., 0., Rotation2d.fromDegrees(0.));
        OTOSOdometry m_odometry = new OTOSOdometry(m_OTOS::getPose2d, initialPose);
    }

    public Pose2d getPose() {
        return m_odometry.getPose();
    }

    /**
     * Call this at the end of every loop
     */
    public void update() {
        m_odometry.updatePose();
    }

    /**
     * Updates the pose every cycle
     */
    @Override
    public void periodic() {
        m_odometry.updatePose();
    }

}