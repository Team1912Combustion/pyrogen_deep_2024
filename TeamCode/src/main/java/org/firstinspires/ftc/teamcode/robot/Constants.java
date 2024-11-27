package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Constants {
    public static class OdometryConstants {
        public final static String sensor_name = "otos_sensor";
        public final static double[] vec_stateStdDevs = {1., 1., .1};
        public final static double[] vec_visionStdDevs = {4., 4., 999.};
        public final static double max_apriltag_poserr = 48.;
    }
    public static class ArmConstants {
        public static final int intake = 0;
        // gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-188-1-ratio-24mm-length-6mm-d-shaft-30-rpm-36mm-gearbox-3-3-5v-encoder/
        // 5281 PPR * (90/360) * (2:1 bevel gear)
        public static final int encoder_max = 5281*2/4;
        public static final int high_basket = 1000;
        public static final int low_basket = 500;
        public static final int threshold = 5;
        public static final double run_speed = 0.2;
    }
    public static class ElevatorConstants {
        public static final int full_in = 0;
        // 581 PPR
        // gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final int full_out = 581;
        public static final int high_basket = 200;
        public static final int low_basket = 100;
        public static final int threshold = 5;
        public static final double run_speed = 0.2;
    }
    public static class IntakeConstants {
        public static final double intake_speed = 1.;
        public static final double output_speed = -1.;
    }

    public static class DriveConstants {
        public static final String front_left_name = "front_left";
        public static final String front_right_name = "front_right";
        public static final String back_left_name = "back_left";
        public static final String back_right_name = "back_right";
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
