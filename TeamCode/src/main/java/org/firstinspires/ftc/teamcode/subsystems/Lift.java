package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants.LiftConstants;
import org.team1912.pyrogen.pyrolib.ftclib.command.SubsystemBase;
import org.team1912.pyrogen.pyrolib.ftclib.controller.PController;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.Motor;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.MotorEx;

public class Lift extends SubsystemBase {
    private final MotorEx m_lift;
    private final Motor.Encoder m_encoder;
    private final Telemetry telemetry;
    private final PController pid;

    public double current_target;

    public Lift(HardwareMap hMap, Telemetry t_telemetry) {
        telemetry = t_telemetry;
        //m_lift = new MotorEx(hMap, LiftConstants.motor_name, Motor.GoBILDA.RPM_30);
        m_lift = new MotorEx(hMap, "arm", Motor.GoBILDA.RPM_30);
        m_lift.setInverted(true);
        m_encoder = m_lift.encoder;
        m_lift.stopAndResetEncoder();
        current_target = 0.;
        m_lift.setRunMode(Motor.RunMode.RawPower);
        m_lift.set(0.);
        m_lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pid = new PController(LiftConstants.kP);
        pid.setTolerance(LiftConstants.threshold);
        pid.setSetPoint(current_target);
    }

    public double get_position() {
        return (double) m_encoder.getPosition();
    }

    public double get_target() {
        return current_target;
    }

    public double get_setpoint() {
        return pid.getSetPoint();
    }

    public double fix_target(double target) {
        return Math.min(LiftConstants.limit_high,
                Math.max(LiftConstants.limit_low, target));
    }

    public void runToPosition(double position) {
        current_target = fix_target(position);
        pid.setSetPoint(current_target);
    }

    public boolean atTarget() {
        return pid.atSetPoint();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void periodic() {
        //if (atBottom()) { m_encoder.reset(); }
        double pid_power =  pid.calculate(get_position());
        m_lift.set(pid_power);
        telemetry.addLine(String.format("lift enc %f power %f \n", get_position(),pid_power));
        telemetry.addLine(String.format("cur tgt %f\n", current_target));
    }

    public void stop() {
        current_target = get_position();
        m_lift.set(0.);
    }
}
