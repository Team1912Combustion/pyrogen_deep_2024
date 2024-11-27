package org.firstinspires.ftc.teamcode.subsystems;

import android.provider.ContactsContract;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.estimator.OTOSPoseEstimator;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.jama.Matrix;
import org.firstinspires.ftc.teamcode.pyrolib.utils.Datalogger;

import org.firstinspires.ftc.teamcode.robot.Constants.OdometryConstants;

import java.util.List;

public class Estimator extends SubsystemBase {

    private final OTOSPoseEstimator m_estimator;
    private final Odometry m_odometry;
    private final Vision m_vision;
    private final Datalogger m_log;

    public Estimator(Odometry odometry, Vision vision) {
        super();
        Matrix stateStdDevs = new Matrix(3, 1);
        Matrix visionMeasurementStdDevs = new Matrix(3, 1);
        for (int i = 0; i < 3; ++i) {
            stateStdDevs.set(i, 0, OdometryConstants.vec_stateStdDevs[i]);
            visionMeasurementStdDevs.set(i, 0, OdometryConstants.vec_visionStdDevs[i]);
        }
        m_vision = vision;
        m_odometry = odometry;
        m_estimator = new OTOSPoseEstimator(
                m_odometry.m_odometry,
                stateStdDevs,
                visionMeasurementStdDevs);
        m_log = new Datalogger("estimator.log");
        m_log.addField("odoX");
        m_log.addField("odoY");
        m_log.addField("odoH");
        m_log.addField("visX");
        m_log.addField("visY");
        m_log.addField("visH");
        m_log.addField("estX");
        m_log.addField("estY");
        m_log.addField("estH");
        m_log.firstLine();
    }

    public Pose2d getPose() {
        return m_estimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        m_estimator.update(m_odometry.getPose());
        List<Pose2d> pose2ds = m_vision.get_robot_pose_list();
        for (Pose2d vispose : pose2ds) {
            // check that apriltag pose is less than 6in (max_apriltag_poserr) from the current
            // pose estimate before adding it
            if (m_odometry.getPose().minus(vispose).getTranslation().getNorm() <
                    OdometryConstants.max_apriltag_poserr) {
                m_estimator.addVisionMeasurement(vispose, m_estimator.getTimestamp());
            }
            Pose2d pose2d = m_odometry.getPose();
            m_log.addField(pose2d.getX());
            m_log.addField(pose2d.getY());
            m_log.addField(pose2d.getRotation().getDegrees());
            m_log.addField(vispose.getX());
            m_log.addField(vispose.getY());
            m_log.addField(vispose.getRotation().getDegrees());
            pose2d = m_estimator.getEstimatedPosition();
            m_log.addField(pose2d.getX());
            m_log.addField(pose2d.getY());
            m_log.addField(pose2d.getRotation().getDegrees());
            m_log.newLine();
        }
    }

}
