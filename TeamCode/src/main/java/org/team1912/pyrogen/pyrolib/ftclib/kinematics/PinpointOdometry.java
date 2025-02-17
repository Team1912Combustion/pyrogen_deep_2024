package org.team1912.pyrogen.pyrolib.ftclib.kinematics;

import org.team1912.pyrogen.pyrolib.GoBildaPinpoint.GoBildaPinpointDriver;
import org.team1912.pyrogen.pyrolib.GoBildaPinpoint.PinpointSensor;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;

public class PinpointOdometry extends Odometry {

    // the suppliers
    public PinpointSensor m_pinpoint;

    // xOffset, yOffset in mm
    public PinpointOdometry(PinpointSensor pinpoint, Pose2d initialPose,
                            double xOffset, double yOffset,
                            GoBildaPinpointDriver.EncoderDirection xDirection,
                            GoBildaPinpointDriver.EncoderDirection yDirection,
                            GoBildaPinpointDriver.GoBildaOdometryPods pods) {
        super(initialPose);
        m_pinpoint = pinpoint;
        m_pinpoint.initialize();
        m_pinpoint.setEncoderResolution(pods);
        m_pinpoint.setOffsets(xOffset, yOffset);
        m_pinpoint.setEncoderDirections(xDirection, yDirection);
    }

    public PinpointOdometry(PinpointSensor pinpoint, Pose2d initialPose) {
        super(initialPose);
        m_pinpoint = pinpoint;
        m_pinpoint.initialize();
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        m_pinpoint.update();
        robotPose = m_pinpoint.getPose2d();
    }

    @Override
    public void updatePose(Pose2d pose) {
        m_pinpoint.setPose2d(pose);
        m_pinpoint.update();
        robotPose = pose;
    }

}
