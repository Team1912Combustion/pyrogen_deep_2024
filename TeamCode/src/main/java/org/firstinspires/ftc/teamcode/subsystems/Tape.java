package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants.TapeConstants;
import org.team1912.pyrogen.pyrolib.ftclib.command.SubsystemBase;
import org.team1912.pyrogen.pyrolib.ftclib.controller.PController;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.Motor;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.MotorEx;

public class Tape extends SubsystemBase {
    private final MotorEx m_tape;
    private final Motor.Encoder m_encoder;
    private final Telemetry telemetry;
    private final PController pid;

    public double current_target;

    public Tape(HardwareMap hMap, Telemetry t_telemetry) {
        telemetry = t_telemetry;
        m_tape = new MotorEx(hMap, TapeConstants.motor_name, Motor.GoBILDA.RPM_30);
        m_tape.setInverted(true);
        m_encoder = m_tape.encoder;
        m_tape.stopAndResetEncoder();
        current_target = 0.;
        m_tape.setRunMode(Motor.RunMode.RawPower);
        m_tape.set(0.);
        m_tape.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pid = new PController(TapeConstants.kP);
        pid.setTolerance(TapeConstants.threshold);
        pid.setSetPoint(current_target);
    }

    public int get_position() {
        return m_encoder.getPosition();
    }

    public double get_target() {
        return current_target;
    }

    public double get_setpoint() {
        return pid.getSetPoint();
    }

    public double fix_target(double target) {
        return Math.min(TapeConstants.limit_high,
                Math.max(TapeConstants.limit_low, target));
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
        m_tape.set(pid_power);
        telemetry.addLine(String.format("tape enc %d power %f \n", get_position(),pid_power));
        telemetry.addLine(String.format("cur ang %f tgt ang %f\n", current_target, get_position()));
    }

    public void stop() {
        current_target = get_position();
        m_tape.set(0.);
    }
}
