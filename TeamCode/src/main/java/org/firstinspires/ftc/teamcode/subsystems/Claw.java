
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private final Servo claw_left;
    private final Servo claw_right;
    private final Telemetry telemetry;
    private double pos_left;
    private double pos_right;
    private boolean is_open;

    public Claw(HardwareMap hMap, Telemetry t_telemetry) {
        claw_left = hMap.get(Servo.class, ClawConstants.left_name);
        claw_right = hMap.get(Servo.class, ClawConstants.right_name);
        claw_right.setDirection(Servo.Direction.REVERSE);
        claw_left.setDirection(Servo.Direction.FORWARD);
        telemetry = t_telemetry;
        pos_left = ClawConstants.init_pos;
        pos_right = ClawConstants.init_pos;
        is_open = false;
        goHold();
        claw_left.setPosition(pos_left);
        claw_right.setPosition(pos_right);
    }

    public void goSafe() {
        pos_left = ClawConstants.left_safe;
        pos_right = ClawConstants.right_safe;
    }
    public void goOpen() {
        pos_left = ClawConstants.left_open;
        pos_right = ClawConstants.right_open;
    }
    public void goHold() {
        pos_left = ClawConstants.left_close;
        pos_right = ClawConstants.right_close;
    }
    public void Toggle() {
        if (is_open) {
            is_open = false;
            goHold();
        } else {
            is_open = true;
            goOpen();
        }
    }

    @Override
    public void periodic() {
        claw_left.setPosition(pos_left);
        claw_right.setPosition(pos_right);
        telemetry.addLine(String.format("claw L/R pos %f:%f\n", pos_left, pos_right));
    }

}
