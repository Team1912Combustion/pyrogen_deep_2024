package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Translation2d;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

public class Constants {
    public static class OdometryConstants {
        public final static String sensor_name = "otos_sensor";
        public final static double[] vec_stateStdDevs = {1., 1., .1};
        public final static double[] vec_visionStdDevs = {4., 4., 999.};
        public final static double max_apriltag_poserr = 48.;
    }

    public static class TapeConstants {
        public static final String motor_name = "tape";
        public final static String limit_name = "tape_limit";
        // 581 PPR
        // gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final int full_out = 100;
        public static final int full_in = 0;
        public static final int limit_high = full_out;
        public static final int limit_low = full_in;
        public static final int threshold = 10;
        // PController P
        public static final double kP = 1.0; // V per tick
    }

    public static class LiftConstants {
        public static final String motor_name = "lift";
        public final static String limit_name = "lift_limit";
        // 581 PPR
        // gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final int full_out = 1600;
        public static final int full_in = 0;
        public static final int limit_high = full_out;
        public static final int limit_low = full_in;
        public static final int threshold = 10;
        // PController P
        public static final double kP = 1.0; // V per tick
    }

    public static class ArmConstants {
        public static final String motor_name = "arm";
        public final static String limit_name = "arm_limit";
        // gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-188-1-ratio-24mm-length-6mm-d-shaft-30-rpm-36mm-gearbox-3-3-5v-encoder/
        // 5281 PPR * (90/360) * (2:1 bevel gear)
        //public static final int tick_90 = (5281*2)/4;   // = 2640
        public static final int tick_90 = 2640;   // = 2640
        public static final double radPerTick = Math.PI * 0.5 / tick_90;
        public static final int pos_limit_high = tick_90 * 9 / 10;
        public static final int pos_limit_low = 0;
        public static final int pos_threshold = 10;
        public static final double angle_threshold = pos_threshold * radPerTick;
        public static final int pos_check = 1100;
        public static final double angle_limit_high = pos_limit_high * radPerTick;
        public static final double angle_limit_low = pos_limit_low * radPerTick;
        public static final int pos_level = 0;
        public static final double angle_level = pos_level * radPerTick;

        public class Sample {
            public static final int pos_high = 2500;
            public static final int pos_mid = 1970;
            public static final int pos_intake = 0;
            public static final double angle_high = pos_high * radPerTick;
            public static final double angle_mid = pos_mid * radPerTick;
            public static final double angle_intake = pos_intake * radPerTick;
        }

        public class Specimen {
            public static final int pos_high = 1590;
            public static final int pos_mid = 790;
            public static final int pos_intake = 320;
            public static final double angle_high = pos_high * radPerTick;
            public static final double angle_mid = pos_mid * radPerTick;
            public static final double angle_intake = pos_intake * radPerTick;
        }

        // PController P
        public static final double kP = 20.0; // V per angle error radians
        // ProfiledController
        public static final double maxVelocity = Math.PI / 2.; // rad/s = 90 deg per second
        public static final double maxAcceleration = maxVelocity / 2.; // rad/s/s = maxVel / 2sec
        // ArmFeedForward
        public static final double vel_radpersec = 0.;
        public static final double kS = 0.; // V
        public static final double maxVoltage = 12.; // = pos_90 * radPerTick;
        public static final double kG = 0.7 / maxVoltage; // V
        public static final double kV = 7.3 / maxVoltage; // V*s/rad
        public static final double kA = 0.17 / maxVoltage; // V*s^2/rad
    }

    public static class ElevatorConstants {
        public static final String motor_name = "elevator";
        public final static String touch_sensor_name = "elevator_touch";
        public static final int full_in = 0;
        // 581 PPR
        // gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final int full_out = 4500;
        public static final int threshold = 100;
        // PID controller
        public static final double kP = .01 ; // percent power (-1/1) per encoder tick error
        // ProfiledPIDcontroller
        public static final double maxVelocity = 4000.; // ticks/s = 1 second full out
        public static final double maxAcceleration = maxVelocity / 1.; // ticks/s/s = full speed in 1 s

        public class Sample {
            public static final int high_goal = 4470;
            public static final int low_goal = 2164;
            public static final int intake = 500;
        }
        public class Specimen {
            public static final int high_goal = 1435;
            public static final int low_goal = 480;
            public static final int intake = 0;
        }
    }

    public static class SpecimenConstants {
        public static final String servo_name = "specimen";
        public static final double init_pos = 0.0;
        public static final double safe_pos = 0.0;
        public static final double hold_pos = 0.35;
    }

    public static class ClawConstants {
        public static final String left_name = "claw_left";
        public static final String right_name = "claw_right";
        public static final double init_pos = 0.0;
        public static final double left_open= 0.05;
        public static final double right_open= 0.05;
        public static final double left_safe= 0.1;
        public static final double right_safe= 0.1;
        public static final double left_close = 0.2;
        public static final double right_close = 0.2;
    }

    public static class IntakeConstants {
        public static final String servo_name = "intake";
        public static final double intake_speed = 1.;
        public static final double output_speed = -1.;
    }

    public static class DriveConstants {
        public static final String front_left_name = "front_left";
        public static final String front_right_name = "front_right";
        public static final String back_left_name = "back_left";
        public static final String back_right_name = "back_right";
        public static Translation2d frontLeftInches = new Translation2d(8,8);
        public static Translation2d backLeftInches = new Translation2d(-8,8);
        public static Translation2d frontRightInches = new Translation2d(8,-8);
        public static Translation2d backRightInches = new Translation2d(-8,-8);
        public static MecanumDriveKinematics kinematics =
                new MecanumDriveKinematics(
                        frontLeftInches,
                        frontRightInches,
                        backLeftInches,
                        backRightInches);
    }

    public static class VisionConstants {
    /**
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
        public final static boolean USE_WEBCAM = true;
        public final static String webcam_name = "webcam";
        public final static Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
        public final static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    }

}
