package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.estimator.OTOSPoseEstimator;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.jama.Matrix;

import org.firstinspires.ftc.teamcode.robot.Constants.OdometryConstants;

import java.util.List;

public class Estimator extends SubsystemBase {

    private final OTOSPoseEstimator m_estimator;
    private final Odometry m_odometry;
    private final Vision m_vision;
    private final Telemetry m_telemetry;

    public Estimator(Odometry odometry, Vision vision, Telemetry telemetry) {
        super();
        Matrix stateStdDevs = new Matrix(3, 1);
        Matrix visionMeasurementStdDevs = new Matrix(3, 1);
        for (int i = 0; i < 3; ++i) {
            stateStdDevs.set(i, 0, OdometryConstants.vec_stateStdDevs[i]);
            visionMeasurementStdDevs.set(i, 0, OdometryConstants.vec_visionStdDevs[i]);
        }
        m_telemetry = telemetry;
        m_vision = vision;
        m_odometry = odometry;
        m_estimator = new OTOSPoseEstimator(
                m_odometry.m_odometry,
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    public Pose2d getPose() {
        return m_estimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        m_estimator.update(m_odometry.getPose());
        List<Pose2d> pose2ds = m_vision.get_robot_pose_list();
        for (Pose2d pose : pose2ds) {
            // check that apriltag pose is less than 6in (max_apriltag_poserr) from the current
            // pose estimate before adding it
            if (m_odometry.getPose().minus(pose).getTranslation().getNorm() <
                    OdometryConstants.max_apriltag_poserr) {
                m_estimator.addVisionMeasurement(pose, m_estimator.getTimestamp());
            }
            m_telemetry.addLine(
                    String.format("Pose %6.1f %6.1f %6.1f",
                            pose.getX(),
                            pose.getY(),
                            pose.getRotation().getDegrees())
            );
            Pose2d pose2d = m_estimator.getEstimatedPosition();
            m_telemetry.addLine(
                    String.format("Esti %6.1f %6.1f %6.1f",
                            pose2d.getX(),
                            pose2d.getY(),
                            pose2d.getRotation().getDegrees())
            );
        }
    }

}
