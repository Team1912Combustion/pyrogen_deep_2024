package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final MotorEx m_arm;
    private final Motor.Encoder m_encoder;
    private int current_target;

    public Arm(HardwareMap hMap, String motorName) {
        m_arm = new MotorEx(hMap, motorName);
        m_encoder = m_arm.encoder;
        m_arm.stopAndResetEncoder();
        m_arm.setRunMode(Motor.RunMode.VelocityControl);
    }

    public int get_position() {
        return m_encoder.getPosition();
    }

    public boolean atBottom() {
        return (Math.abs(m_encoder.getPosition()) < ArmConstants.threshold);
    }
    public boolean atTop() {
        return (Math.abs(m_encoder.getPosition() - ArmConstants.encoder_max) < ArmConstants.threshold);
    }
    public boolean atTarget() {
        return (Math.abs(m_encoder.getPosition() - current_target) < ArmConstants.threshold);
    }

    public boolean safeToMove() {
        return (! atBottom() && ! atTop());
    }

    public int fix_target(int target) {
        return Math.min(ArmConstants.encoder_max, Math.max(ArmConstants.intake, target));
    }

    public void runToPosition(int target) {
        current_target = fix_target(target);
        m_arm.setRunMode(Motor.RunMode.PositionControl);
        m_arm.setTargetPosition(current_target);
        m_arm.set(ArmConstants.run_speed);
    }

    public void move(double speed) {
        m_arm.setRunMode(Motor.RunMode.VelocityControl);
        m_arm.set(speed);
    }

    public void stop() {
        m_arm.set(0.);
    }
}
