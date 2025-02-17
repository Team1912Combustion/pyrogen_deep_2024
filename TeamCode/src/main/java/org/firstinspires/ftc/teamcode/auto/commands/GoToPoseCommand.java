package org.firstinspires.ftc.teamcode.auto.commands;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.team1912.pyrogen.pyrolib.ftclib.controller.PIDController;
import org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Transform2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Translation2d;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.wpilibkinematics.MecanumDriveMotorVoltages;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import org.team1912.pyrogen.pyrolib.ftclib.util.MathUtil;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to drive to a Pose2d
 * with a mecanum drive.
 *
 * <p>The command handles Velocity PID calculations internally. This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID functionality, returning only the raw wheel speeds from the PID
 * controllers.
 *
 * <p>The robot angle controller goes to the angle given in the final state of the trajectory.
 */

@SuppressWarnings({"PMD.TooManyFields", "MemberName"})
public class GoToPoseCommand extends CommandBase {
    private final ElapsedTime m_timer;
    private final Pose2d m_finalPose;
    private final double m_runtime;
    private final double m_timeout;
    private Pose2d m_startPose;
    private final boolean m_usePID;

    private double m_error;
    private double distError;

    private final Supplier<Pose2d> m_pose;
    private final MecanumDriveKinematics m_kinematics;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;
    private final double m_maxWheelVelocityMetersPerSecond;
    private final PIDController m_frontLeftController;
    private final PIDController m_rearLeftController;
    private final PIDController m_frontRightController;
    private final PIDController m_rearRightController;
    private final Supplier<MecanumDriveWheelSpeeds> m_currentWheelSpeeds;
    private final Consumer<MecanumDriveMotorVoltages> m_outputDriveVoltages;
    private final Consumer<MecanumDriveWheelSpeeds> m_outputWheelSpeeds;

    /**
     * Constructs a new MecanumControllerCommand that when executed will drive to the
     * provided final Pose2d. PID control is handled internally. Outputs are scaled from -12 to
     * 12 as a voltage output to the motor.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path
     * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * <p>Note 2: The rotation controller will calculate the rotation based on the final pose.
     *
     * @param finalPose                      The end goal Pose2d
     * @param pose                            A function that supplies the robot pose - use one of
     *                                        the odometry classes to provide this.
     * @param kinematics                      The kinematics for the robot drivetrain.
     * @param xController                     The Trajectory Tracker PID controller
     *                                        for the robot's x position.
     * @param yController                     The Trajectory Tracker PID controller
     *                                        for the robot's y position.
     * @param thetaController                 The Trajectory Tracker PID controller
     *                                        for angle for the robot.
     * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
     * @param frontLeftController             The front left wheel velocity PID.
     * @param rearLeftController              The rear left wheel velocity PID.
     * @param frontRightController            The front right wheel velocity PID.
     * @param rearRightController             The rear right wheel velocity PID.
     * @param currentWheelSpeeds              A MecanumDriveWheelSpeeds object containing
     *                                        the current wheel speeds.
     * @param outputDriveVoltages             A MecanumDriveMotorVoltages object containing
     *                                        the output motor voltages.
     */

    @SuppressWarnings({"PMD.ExcessiveParameterList", "ParameterName"})
    public GoToPoseCommand(Pose2d finalPose,
                           double runtime,
                           double timeout,
                           double error,
                           Supplier<Pose2d> pose,
                           MecanumDriveKinematics kinematics,
                           PIDController xController,
                           PIDController yController,
                           ProfiledPIDController thetaController,
                           double maxWheelVelocityMetersPerSecond,
                           PIDController frontLeftController,
                           PIDController rearLeftController,
                           PIDController frontRightController,
                           PIDController rearRightController,
                           Supplier<MecanumDriveWheelSpeeds> currentWheelSpeeds,
                           Consumer<MecanumDriveMotorVoltages> outputDriveVoltages) {
        m_finalPose = finalPose;
        m_runtime = runtime;
        m_timeout = timeout;
        m_pose = pose;
        m_kinematics = kinematics;
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;
        m_frontLeftController = frontLeftController;
        m_rearLeftController = rearLeftController;
        m_frontRightController = frontRightController;
        m_rearRightController = rearRightController;
        m_currentWheelSpeeds = currentWheelSpeeds;
        m_outputDriveVoltages = outputDriveVoltages;
        m_outputWheelSpeeds = null;
        m_usePID = true;
        m_error = error;
        m_timer = new ElapsedTime();
    }

    /**
     * Constructs a new GoToPoseCommand that when executed will drive to the provided Pose2d.
     * The user should implement a velocity PID on the desired output wheel velocities.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path -
     * this is left to the user, since it is not appropriate for paths with non-stationary end-states.
     *
     * <p>Note2: The rotation controller will calculate the rotation based on the final pose
     * in the trajectory, not the poses at each time step.
     *
     * @param finalPose                       The final Pose2d.
     * @param pose                            A function that supplies the robot pose - use one of
     *                                        the odometry classes to provide this.
     * @param kinematics                      The kinematics for the robot drivetrain.
     * @param xController                     The Trajectory Tracker PID controller
     *                                        for the robot's x position.
     * @param yController                     The Trajectory Tracker PID controller
     *                                        for the robot's y position.
     * @param thetaController                 The Trajectory Tracker PID controller
     *                                        for angle for the robot.
     * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
     * @param outputWheelSpeeds               A MecanumDriveWheelSpeeds object containing
     *                                        the output wheel speeds.
     */

    @SuppressWarnings({"PMD.ExcessiveParameterList", "ParameterName"})
    public GoToPoseCommand(Pose2d finalPose,
                           double runtime,
                           double timeout,
                           double error,
                           Supplier<Pose2d> pose,
                           MecanumDriveKinematics kinematics,
                           PIDController xController,
                           PIDController yController,
                           ProfiledPIDController thetaController,
                           double maxWheelVelocityMetersPerSecond,
                           Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds) {
        m_finalPose = finalPose;
        m_runtime = runtime;
        m_timeout = timeout;
        m_pose = pose;
        m_kinematics = kinematics;
        m_startPose = m_pose.get();
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;
        m_frontLeftController = null;
        m_rearLeftController = null;
        m_frontRightController = null;
        m_rearRightController = null;
        m_currentWheelSpeeds = null;
        m_outputWheelSpeeds = outputWheelSpeeds;
        m_outputDriveVoltages = null;
        m_usePID = false;
        m_error = error;
        m_timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        m_startPose = m_pose.get();
        m_timer.reset();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.seconds();

        Pose2d desiredPose = interpolate(m_startPose, m_finalPose,curTime / m_runtime);
        distError = m_finalPose.minus(m_pose.get()).getTranslation().getNorm();

        double targetXVel = m_xController.calculate(
                m_pose.get().getTranslation().getX(),
                desiredPose.getTranslation().getX());

        double targetYVel = m_yController.calculate(
                m_pose.get().getTranslation().getY(),
                desiredPose.getTranslation().getY());

        // The robot will go to the desired rotation of the final pose in the trajectory,
        // not following the poses at individual states.
        double targetAngularVel = m_thetaController.calculate(
                m_pose.get().getRotation().getRadians(),
                m_finalPose.getRotation().getRadians());

        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(targetXVel, targetYVel, targetAngularVel);
        MecanumDriveWheelSpeeds targetWheelSpeeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);
        targetWheelSpeeds.normalize(m_maxWheelVelocityMetersPerSecond);
        double frontLeftSpeedSetpoint = targetWheelSpeeds.frontLeftMetersPerSecond;
        double rearLeftSpeedSetpoint = targetWheelSpeeds.rearLeftMetersPerSecond;
        double frontRightSpeedSetpoint = targetWheelSpeeds.frontRightMetersPerSecond;
        double rearRightSpeedSetpoint = targetWheelSpeeds.rearRightMetersPerSecond;

        double frontLeftOutput;
        double rearLeftOutput;
        double frontRightOutput;
        double rearRightOutput;

        if (m_usePID) {
            frontLeftOutput = m_frontLeftController.calculate(
                    m_currentWheelSpeeds.get().frontLeftMetersPerSecond,
                    frontLeftSpeedSetpoint);
            rearLeftOutput = m_rearLeftController.calculate(
                    m_currentWheelSpeeds.get().rearLeftMetersPerSecond,
                    rearLeftSpeedSetpoint);
            frontRightOutput = m_frontRightController.calculate(
                    m_currentWheelSpeeds.get().frontRightMetersPerSecond,
                    frontRightSpeedSetpoint);
            rearRightOutput = m_rearRightController.calculate(
                    m_currentWheelSpeeds.get().rearRightMetersPerSecond,
                    rearRightSpeedSetpoint);
            m_outputDriveVoltages.accept(new MecanumDriveMotorVoltages(
                    frontLeftOutput,
                    frontRightOutput,
                    rearLeftOutput,
                    rearRightOutput));
        } else {
            m_outputWheelSpeeds.accept(new MecanumDriveWheelSpeeds(
                    frontLeftSpeedSetpoint,
                    frontRightSpeedSetpoint,
                    rearLeftSpeedSetpoint,
                    rearRightSpeedSetpoint));
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.seconds() > m_timeout ||
               distError < m_error;
    }

    public Pose2d interpolate(Pose2d startPose, Pose2d endPose, double t) {
        Transform2d delta =  endPose.minus(startPose).times(MathUtil.clamp(t, 0.0, 1.0));
        return startPose.transformBy(delta);
    }
}