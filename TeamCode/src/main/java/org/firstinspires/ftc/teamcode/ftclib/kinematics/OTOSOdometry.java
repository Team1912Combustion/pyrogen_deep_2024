package org.firstinspires.ftc.teamcode.ftclib.kinematics;

import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Twist2d;

import java.util.function.Supplier;

public class OTOSOdometry extends Odometry {

    private double prevX, prevY;
    private Rotation2d prevAngle;
    private Translation2d m_offset;

    private double m_xOffset, m_yOffset;

    // the suppliers
    Supplier<Pose2d> m_OTOSSensor;

    public OTOSOdometry(Supplier<Pose2d> OTOSSensor, Pose2d initialPose, double xOffset, double yOffset) {
        super(initialPose);
        m_OTOSSensor = OTOSSensor;
        m_offset = new Translation2d(xOffset, yOffset);
    }

    public OTOSOdometry(Supplier<Pose2d> OTOSSensor, Pose2d initialPose) {
        super(initialPose);
        m_OTOSSensor = OTOSSensor;
        m_offset = new Translation2d(0., 0.);
    }
    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        update(m_OTOSSensor.get());
    }
    @Override
    public void updatePose(Pose2d pose) {
        prevX = 0.;
        prevY = 0.;
        prevAngle = pose.getRotation();
        robotPose = pose;
    }

    public Pose2d update(Pose2d new_Pose2d) {

        double X = new_Pose2d.getX();
        double Y = new_Pose2d.getY();
        Rotation2d angle = new_Pose2d.getRotation();

        Translation2d newOffset = m_offset.rotateBy(angle);

        double dx = X - prevX;
        double dy = Y - prevY;
        double dw = (angle.minus(prevAngle).getRadians());
        Twist2d twist2d = new Twist2d(dx, dy, dw);
        Pose2d newPose = robotPose.exp(twist2d);

        prevX = X;
        prevY = Y;
        prevAngle = angle;
        robotPose = new Pose2d(newPose.getTranslation(), angle);
        return robotPose;
    }

    public Pose2d getPose() {
        return robotPose;
    }
}
