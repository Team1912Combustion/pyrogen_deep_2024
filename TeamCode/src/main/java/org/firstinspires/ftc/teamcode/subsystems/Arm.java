package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public double current_target;
    private Telemetry m_telemetry;

    public Arm(HardwareMap hMap, String motorName, Telemetry telemetry) {
        m_telemetry = telemetry;
        m_arm = new MotorEx(hMap, motorName, Motor.GoBILDA.RPM_30);
        m_arm.setInverted(true);
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
        pid.setSetPoint(current_target);
    }

    public int get_position() {
        return m_encoder.getPosition();
    }

    public double get_angle() {
        return (double) get_position() * (Math.PI * 0.5) / ArmConstants.tick_90;
    }

    public double fix_target(double target) {
        return  Math.min(ArmConstants.angle_limit_high,
                Math.max(ArmConstants.angle_limit_low, target));
    }

    public void runToAngle(double angle) {
        current_target = fix_target(angle);
        pid.setSetPoint(current_target);
    }

    public boolean atTarget() {
        return pid.atSetPoint();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void periodic() {
        double pid_power =  pid.calculate(get_angle());
        double feed_power = feedforward.calculate(get_angle(),ArmConstants.vel_radpersec);
        double power =  pid_power + feed_power;
        m_arm.set(power);
        m_telemetry.addLine(String.format("arm enc %d power %f\n", get_position(),power));
        m_telemetry.addLine(String.format("arm pid %f feed %f\n", pid_power, feed_power));
        m_telemetry.addLine(String.format("cur ang %f tgt ang %f\n", current_target, get_angle()));
    }

    public void stop() {
        current_target = get_angle();
        m_arm.set(0.);
    }
}
