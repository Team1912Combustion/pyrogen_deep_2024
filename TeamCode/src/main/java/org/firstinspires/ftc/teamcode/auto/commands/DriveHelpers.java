/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.team1912.pyrogen.pyrolib.OTOS.OTOSSensor;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandOpMode;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Transform2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Translation2d;

public class DriveHelpers {

//  CommandOpMode opMode;
//  Telemetry telemetry;
//  Odometry odometry;
//  Drive drive;
//
//  private double          headingError  = 0;
//
//  // These variable are declared here (as class members) so they can be updated in various methods,
//  // but still be displayed by sendTelemetry()
//  private double  targetHeading = 0;
//  private double  driveSpeed    = 0;
//  private double  turnSpeed     = 0;
//
//  // These constants define the desired driving/control characteristics
//  // They can/should be tweaked to suit the specific robot drive train.
//  static final double     DISTANCE_THRESHOLD      = 1.0 ;    // How close must the position be to the final position
//                                                             // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
//  static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
//  static final double     TURN_SPEED              = 0.2;     // Max turn speed to limit turn rate.
//  static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
//                                                             // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
//  // Define the Proportional control coefficient (or GAIN) for "heading control".
//  // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
//  // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
//  // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
//  static final double     P_TURN_GAIN            = 0.1;     // Larger is more responsive, but also less stable.
//  static final double     P_DRIVE_GAIN           = 0.1;     // Larger is more responsive, but also less stable.
//
//  public DriveHelpers(CommandOpMode o_opMode, Telemetry t_telemetry)  {
//      opMode = o_opMode;
//      telemetry = t_telemetry;
//  }
//
//  public void init(Drive d_drive, Odometry o_odometry, HardwareMap hardwareMap, Telemetry telemetry) {
//
//      odometry = o_odometry;
//      drive = d_drive;
//
//      // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
//      drive.drive(0.,0.,0.);
//      odometry.update(new Pose2d(0.,0., Rotation2d.fromDegrees(0.)));
//
//      resetHeading();
//  }
//
//  /*
//   * ====================================================================================================
//   * Driving "Helper" functions are below this line.
//   * These provide the high and low level methods that handle driving straight and turning.
//   * ====================================================================================================
//   */
//
//  // **********  HIGH Level driving functions.  ********************
//
//  /**
//  *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
//  *  Move will stop if either of these conditions occur:
//  *  1) Move gets to the desired position
//  *  2) Driver stops the OpMode running.
//  *
//  * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
//  * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
//  * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
//  *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//  *                   If a relative angle is required, add/subtract from the current robotHeading.
//  */
//
//  public void runToTarget(double maxDriveSpeed,
//                          double maxTurnSpeed,
//                          double timeoutSeconds,
//                          Pose2d targetPose2d) {
//
//      // Ensure that the OpMode is still active
//      if (opMode.opModeIsActive()) {
//
//          ElapsedTime timer = new ElapsedTime();
//          timer.reset();
//
//          Translation2d vectorToTarget =
//                  targetPose2d.minus(odometry.getPose()).getTranslation();
//
//          double distanceToTarget = vectorToTarget.getNorm();
//
//          // keep looping while we are still active, and BOTH motors are running.
//          while (opMode.opModeIsActive()
//                  && timer.seconds() < timeoutSeconds
//                  && distanceToTarget > DISTANCE_THRESHOLD) {
//
//              // Determine required steering to keep on heading
//              turnSpeed = getSteeringCorrection(getHeading(), P_DRIVE_GAIN);
//              driveSpeed = distanceToTarget *
//
//              // Apply the turning correction to the current driving speed.
//              drive.drive(driveSpeed, turnSpeed);
//
//              // Display drive status for the driver.
//              sendTelemetry(true);
//          }
//
//          // Stop all motion & Turn off RUN_TO_POSITION
//          moveRobot(0, 0);
//          leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//          rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//          leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//          rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      }
//  }
//
//  public void strafeStraight(double maxDriveSpeed,
//                            double distance,
//                            double heading) {
//
//      // Ensure that the OpMode is still active
//      if (opMode.opModeIsActive()) {
//
//          // Determine new target position, and pass to motor controller
//          int moveCounts = (int)(distance * COUNTS_PER_INCH);
//          int leftFrontTarget  =  leftFront.getCurrentPosition() + moveCounts;
//          int leftBackTarget   =   leftBack.getCurrentPosition() - moveCounts;
//          int rightFrontTarget = rightFront.getCurrentPosition() - moveCounts;
//          int rightBackTarget  =  rightBack.getCurrentPosition() + moveCounts;
//
//          // Set Target FIRST, then turn on RUN_TO_POSITION
//          leftFront.setTargetPosition(leftFrontTarget);
//          rightFront.setTargetPosition(rightFrontTarget);
//          leftBack.setTargetPosition(leftBackTarget);
//          rightBack.setTargetPosition(rightBackTarget);
//
//          leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//          rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//          leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//          rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//          // Set the required driving speed  (must be positive for RUN_TO_POSITION)
//          // Start driving straight, and then enter the control loop
//          maxDriveSpeed = Math.abs(maxDriveSpeed);
//          strafeRobot(maxDriveSpeed, 0);
//
//          // keep looping while we are still active, and BOTH motors are running.
//          while (opMode.opModeIsActive() &&
//                  (leftFront.isBusy() && rightFront.isBusy())) {
//
//              // Determine required steering to keep on heading
//              turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//              // if driving in reverse, the motor correction also needs to be reversed
//              if (distance < 0) turnSpeed *= -1.0;
//
//              // Apply the turning correction to the current driving speed.
//              strafeRobot(driveSpeed, turnSpeed);
//
//              // Display drive status for the driver.
//              sendTelemetry(true);
//          }
//
//          // Stop all motion & Turn off RUN_TO_POSITION
//          moveRobot(0, 0);
//          leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//          rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//          leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//          rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      }
//  }
//
//
//  /**
//   *  Spin on the central axis to point in a new direction.
//   *  <p>
//   *  Move will stop if either of these conditions occur:
//   *  <p>
//   *  1) Move gets to the heading (angle)
//   *  <p>
//   *  2) Driver stops the OpMode running.
//   *
//   * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
//   * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
//   *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//   *              If a relative angle is required, add/subtract from current heading.
//   */
//  public void turnToHeading(double maxTurnSpeed, double heading) {
//
//      // Run getSteeringCorrection() once to pre-calculate the current error
//      getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//      // keep looping while we are still active, and not on heading.
//      while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
//
//          // Determine required steering to keep on heading
//          turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//
//          // Clip the speed to the maximum permitted value.
//          turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
//
//          // Pivot in place by applying the turning correction
//          moveRobot(0, turnSpeed);
//
//          // Display drive status for the driver.
//          sendTelemetry(false);
//      }
//
//      // Stop all motion;
//      moveRobot(0, 0);
//  }
//
//  /**
//   *  Obtain & hold a heading for a finite amount of time
//   *  <p>
//   *  Move will stop once the requested time has elapsed
//   *  <p>
//   *  This function is useful for giving the robot a moment to stabilize its heading between movements.
//   *
//   * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
//   * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
//   *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//   *                   If a relative angle is required, add/subtract from current heading.
//   * @param holdTime   Length of time (in seconds) to hold the specified heading.
//   */
//  public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
//
//      ElapsedTime holdTimer = new ElapsedTime();
//      holdTimer.reset();
//
//      // keep looping while we have time remaining.
//      while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
//          // Determine required steering to keep on heading
//          turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//
//          // Clip the speed to the maximum permitted value.
//          turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
//
//          // Pivot in place by applying the turning correction
//          moveRobot(0, turnSpeed);
//
//          // Display drive status for the driver.
//          sendTelemetry(false);
//      }
//
//      // Stop all motion;
//      moveRobot(0, 0);
//  }
//
//  // **********  LOW Level driving functions.  ********************
//
//  /**
//   * Use a Proportional Controller to determine how much distance correction is required.
//   *
//   * @param distanceError        The desired absolute heading (relative to last heading reset)
//   * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
//   * @return                      Turning power needed to get to required heading.
//   */
//  public double getDistanceCorrection(double desiredHeading, double proportionalGain) {
//      targetHeading = desiredHeading;  // Save for telemetry
//
//      // Determine the heading current error
//      headingError = targetHeading - getHeading();
//
//      // Normalize the error to be within +/- 180 degrees
//      while (headingError > 180)  headingError -= 360;
//      while (headingError <= -180) headingError += 360;
//
//      // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
//      return Range.clip(headingError * proportionalGain, -1, 1);
//  }
//
//  /**
//   * Use a Proportional Controller to determine how much steering correction is required.
//   *
//   * @param desiredHeading        The desired absolute heading (relative to last heading reset)
//   * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
//   * @return                      Turning power needed to get to required heading.
//   */
//  public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
//      targetHeading = desiredHeading;  // Save for telemetry
//
//      // Determine the heading current error
//      headingError = targetHeading - getHeading();
//
//      // Normalize the error to be within +/- 180 degrees
//      while (headingError > 180)  headingError -= 360;
//      while (headingError <= -180) headingError += 360;
//
//      // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
//      return Range.clip(headingError * proportionalGain, -1, 1);
//  }
//
//  /**
//   * Take separate drive (fwd/rev) and turn (right/left) requests,
//   * combines them, and applies the appropriate speed commands to the left and right wheel motors.
//   * @param drive forward motor speed
//   * @param turn  clockwise turning motor speed.
//   */
//  public void moveRobot(double drive, double turn) {
//      driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
//      turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.
//
//      leftSpeed  = drive - turn;
//      rightSpeed = drive + turn;
//
//      // Scale speeds down if either one exceeds +/- 1.0;
//      double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//      if (max > 1.0)
//      {
//          leftSpeed /= max;
//          rightSpeed /= max;
//      }
//
//      leftFront.setPower(leftSpeed);
//      rightFront.setPower(rightSpeed);
//      leftBack.setPower(leftSpeed);
//      rightBack.setPower(rightSpeed);
//  }
//
//  /**
//   * Take separate drive (fwd/rev) and turn (right/left) requests,
//   * combines them, and applies the appropriate speed commands to the left and right wheel motors.
//   * @param strafe right motor speed
//   * @param turn  clockwise turning motor speed.
//   */
//  public void strafeRobot(double strafe, double turn) {
//      driveSpeed = strafe;     // save this value as a class member so it can be used by telemetry.
//      turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.
//
//      double leftFrontSpeed  =   strafe - turn;
//      double rightBackSpeed  =   strafe + turn;
//      double leftBackSpeed   = - strafe - turn;
//      double rightFrontSpeed = - strafe + turn;
//
//      // Scale speeds down if either one exceeds +/- 1.0;
//      double max = Math.max(
//              Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed)),
//              Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed))
//              );
//
//      if (max > 1.0)
//      {
//          leftFrontSpeed /= max;
//          rightBackSpeed /= max;
//          leftBackSpeed /= max;
//          rightFrontSpeed /= max;
//      }
//
//      leftFront.setPower(leftFrontSpeed);
//      rightBack.setPower(rightBackSpeed);
//      leftBack.setPower(leftBackSpeed);
//      rightFront.setPower(rightFrontSpeed);
//  }
//
//  /**
//   *  Display the various control parameters while driving
//   *
//   * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
//   */
//  private void sendTelemetry(boolean straight) {
//
//      if (straight) {
//          telemetry.addData("Motion", "Drive Straight");
//          telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
//          telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFront.getCurrentPosition(),
//                  rightFront.getCurrentPosition());
//      } else {
//          telemetry.addData("Motion", "Turning");
//      }
//
//      telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
//      telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
//      telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
//      telemetry.update();
//  }
//
//  /**
//   * read the Robot heading directly from the IMU (in degrees)
//   */
//  public double getHeading() {
//      return odometry.getPose().getRotation().getDegrees();
//  }
//  public void resetHeading() {
//      Pose2d pose = odometry.getPose();
//      Pose2d newpose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0.));
//      odometry.update(newpose);
//  }
}
