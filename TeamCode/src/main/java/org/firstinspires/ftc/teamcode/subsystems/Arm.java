package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.team1912.pyrogen.pyrolib.ftclib.command.SubsystemBase;
import org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller.ArmFeedforward;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.Motor;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.MotorEx;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final MotorEx m_arm;
    private final Motor.Encoder m_encoder;
    private final Telemetry telemetry;
    //private final DigitalChannel touchSensor;

    private final ProfiledPIDController pid;
    private final ArmFeedforward feedforward;

    public double current_target;

    public Arm(HardwareMap hMap, Telemetry t_telemetry) {
        telemetry = t_telemetry;
        //touchSensor = hMap.get(DigitalChannel.class, Constants.ArmConstants.limit_name);
        //touchSensor.setMode(DigitalChannel.Mode.INPUT);
        m_arm = new MotorEx(hMap, ArmConstants.motor_name, Motor.GoBILDA.RPM_30);
        m_arm.setInverted(true);
        m_encoder = m_arm.encoder;
        m_arm.stopAndResetEncoder();
        current_target = get_angle();
        m_arm.setRunMode(Motor.RunMode.RawPower);
        m_arm.set(0.);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                ArmConstants.maxVelocity,
                ArmConstants.maxAcceleration);
        pid = new ProfiledPIDController(ArmConstants.kP,0.,0., constraints);
        pid.setTolerance(ArmConstants.angle_threshold);
        feedforward = new ArmFeedforward(
                ArmConstants.kS,
                ArmConstants.kG,
                ArmConstants.kV,
                ArmConstants.kA);
        pid.setGoal(current_target);
    }

    //public boolean atBottom() {
    //    return touchSensor.getState() ;
   // }

    public int get_position() {
        return m_encoder.getPosition();
    }

    public double get_target() {
        return current_target;
    }

    public double get_setpoint() {
        return pid.getSetpoint().position;
    }

    public double get_angle() {
        return (double) get_position() * ArmConstants.radPerTick;
    }

    public double old_fix_target(double target) {
        return Math.min(ArmConstants.angle_limit_high,
               Math.max(ArmConstants.angle_limit_low, target));
    }

    public double fix_target(double target) {
        return Math.min(ArmConstants.angle_limit_high,
                Math.max(ArmConstants.angle_limit_low, target));
    }

    public void runToAngle(double angle) {
        current_target = fix_target(angle);
        pid.setGoal(current_target);
    }

    public boolean atTarget() {
        return pid.atGoal();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void periodic() {
        //if (atBottom()) { m_encoder.reset(); }
        double pid_power =  pid.calculate(get_angle());
        double feed_power = feedforward.calculate(get_angle(),ArmConstants.vel_radpersec);
        double power =  pid_power + feed_power;
        m_arm.set(power);
        telemetry.addLine(String.format("arm enc %d power %f", get_position(),power));
        telemetry.addLine(String.format("arm pid %f feed %f", pid_power, feed_power));
        telemetry.addLine(String.format("cur ang %f tgt ang %f", current_target, get_angle()));
        //telemetry.update();
    }

    public void stop() {
        current_target = get_angle();
        m_arm.set(0.);
    }
}
