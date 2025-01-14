
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.OTOS.OTOSSensor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.kinematics.OTOSOdometry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

public class AutoDrive {

	private LinearOpMode opMode;
    private Odometry odo;
    private OTOSSensor imu;
	private Drive drive;
    private Telemetry telemetry;

    private double          headingError  = 0;

    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  strafeSpeed     = 0;
    private double  xTarget       = 0;
    private double  xCurrent      = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     MIN_SPEED               = 0.05;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    static final double     P_TURN_GAIN             = 0.002;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN            = 0.03;     // Larger is more responsive, but also less stable.

    public void init(LinearOpMode m_opMode, Drive m_drive, Odometry m_odometry, Telemetry m_telemetry) {
	    drive = m_drive;
	    odo = m_odometry;
        imu = odo.m_otosOdometry.m_OTOSSensor;
        opMode = m_opMode;
        telemetry = m_telemetry;
    }

    // **********  HIGH Level driving functions.  ********************

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            Pose2d startPose = imu.getPose2d();
            xCurrent = imu.getPose2d().minus(startPose).getTranslation().getNorm();
            xTarget = Math.abs(distance);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(Math.signum(driveSpeed)*MIN_SPEED, 0, 0);
            sendTelemetry(true);
            double myDriveSpeed = driveSpeed;
            if (distance < 0) {
                myDriveSpeed = -1.0 * driveSpeed;
            }

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && xCurrent < xTarget) {
                xCurrent = imu.getPose2d().minus(startPose).getTranslation().getNorm();

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                strafeSpeed = 0.;

                // Apply the turning correction to the current driving speed.
                moveRobot(myDriveSpeed, turnSpeed, strafeSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);
        }
    }

    public void rampStraight(double maxDriveSpeed,
                              double minDriveSpeed,
                             double rampDist,
                             double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            Pose2d startPose = imu.getPose2d();
            xCurrent = imu.getPose2d().minus(startPose).getTranslation().getNorm();
            xTarget = Math.abs(distance);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            minDriveSpeed = Math.abs(minDriveSpeed);
            double dir = 1.;
            if (distance < 0.) dir = -1.;
            double myDriveSpeed = dir*minDriveSpeed;
            moveRobot(myDriveSpeed, 0, 0);
            sendTelemetry(true);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && xCurrent < xTarget) {
                xCurrent = imu.getPose2d().minus(startPose).getTranslation().getNorm();

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                strafeSpeed = 0.;

                myDriveSpeed = dir * rampSpeed(minDriveSpeed, maxDriveSpeed,
                        rampDist, xCurrent, xTarget);
                // Apply the turning correction to the current driving speed.
                moveRobot(myDriveSpeed, turnSpeed, strafeSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);
        }
    }

    private double rampSpeed(double minSpeed, double maxSpeed,
                             double ramp, double travel, double distance) {
        if (travel < ramp)  {
            return minSpeed +
                    ( (double) travel / (double) ramp )
                            * (maxSpeed - minSpeed);
        } else if ( (Math.abs(distance) - travel) < ramp) {
            return minSpeed +
                    ( (double) (Math.abs(distance)-travel) / (double) ramp )
                            * (maxSpeed - minSpeed);
        } else {
            return maxSpeed;
        }
    }

    public void strafeStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            Pose2d startPose = imu.getPose2d();
            xCurrent = imu.getPose2d().minus(startPose).getTranslation().getNorm();
            xTarget = Math.abs(distance);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(0, 0, Math.signum(driveSpeed)*MIN_SPEED);
            sendTelemetry(true);
            double strafeSpeed = driveSpeed;
            if (distance < 0) {
                strafeSpeed = -1.0 * driveSpeed;
            }

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && xCurrent < xTarget) {
                xCurrent = imu.getPose2d().minus(startPose).getTranslation().getNorm();

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                driveSpeed = 0.;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed, strafeSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, 0);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, 0);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param fwd forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double fwd, double turn, double strafe) {
        driveSpeed = fwd;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.
        strafeSpeed  = strafe;      // save this value as a class member so it can be used by telemetry.
        drive.drive(-1.*strafe, -1.*fwd, -1.*turn);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        telemetry.addData("IMU Pose X:Y:heading",  "%5.2f : %5.2f : %5.2f",
                imu.getPose2d().getX(), imu.getPose2d().getY(), imu.getPose2d().getHeading());
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Robot  Dist : Target Dist ",  "%5.2f : %5.2f",
                    xCurrent, xTarget);
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f",
                headingError, turnSpeed);
        telemetry.addData("Drive drive:turn:strafe ", "%5.2f : %5.2f : %5.2f",
                driveSpeed, turnSpeed, strafeSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        return imu.getPose2d().getRotation().getDegrees();
    }
}
