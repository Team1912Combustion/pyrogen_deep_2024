package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.controller.PController;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.controller.wpilibcontroller.ArmFeedforward;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final MotorEx m_arm;
    private final Motor.Encoder m_encoder;
    private final PController pid;
    private final ArmFeedforward feedforward;
    private double current_target;

    public Arm(HardwareMap hMap, String motorName) {
        m_arm = new MotorEx(hMap, motorName);
        m_encoder = m_arm.encoder;
        m_arm.stopAndResetEncoder();
        current_target = get_angle();
        m_arm.setRunMode(Motor.RunMode.RawPower);
        m_arm.set(0.);
        pid = new PController(ArmConstants.kP);
        pid.setTolerance(ArmConstants.angle_threshold);
        feedforward = new ArmFeedforward(
                ArmConstants.kS,
                ArmConstants.kG,
                ArmConstants.kV,
                ArmConstants.kA);
    }

    public int get_position() {
        return m_encoder.getPosition();
    }

    public double get_angle() {
        return (double) get_position() * (Math.PI * 0.5) / ArmConstants.pos_90;
    }

    public double fix_target(double target) {
        return  Math.min(ArmConstants.angle_90,
                Math.max(ArmConstants.angle_intake, target));
    }

    public void runToPosition(double target) {
        current_target = fix_target(target);
        pid.setSetPoint(current_target);
    }

    public boolean atTarget() {
        return pid.atSetPoint();
    }

    @Override
    public void periodic() {
        double power =  pid.calculate(get_angle()) +
                feedforward.calculate(get_angle(),ArmConstants.vel_radpersec);
        m_arm.set(power);
    }

    public void stop() {
        m_arm.set(0.);
    }
}
