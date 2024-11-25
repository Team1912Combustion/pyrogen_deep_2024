package org.firstinspires.ftc.teamcode.robot;

public class Constants {
    public static class ArmConstants {
        public static final int intake = 0;
        // gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-188-1-ratio-24mm-length-6mm-d-shaft-30-rpm-36mm-gearbox-3-3-5v-encoder/
        // 5281 PPR * (90/360) * (2:1 bevel gear)
        public static final int encoder_max = 5281*2/4;
        public static final int high_basket = 1000;
        public static final int low_basket = 500;
        public static final int threshold = 50;
    }
    public static class ElevatorConstants {
        public static final int full_in = 0;
        // 581 PPR
        // gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final int full_out = 581;
        public static final int high_basket = 200;
        public static final int half_out = 100;
        public static final int low_basket = 100;
        public static final int threshold = 20;
    }
    public static class IntakeConstants {
        public static final double intake_speed = 1.;
        public static final double output_speed = -1.;
    }
}
