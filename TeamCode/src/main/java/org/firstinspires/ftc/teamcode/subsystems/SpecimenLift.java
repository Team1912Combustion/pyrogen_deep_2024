
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Constants.LiftConstants;
import org.team1912.pyrogen.pyrolib.ftclib.command.SubsystemBase;
import org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.Motor;
import org.team1912.pyrogen.pyrolib.ftclib.hardware.motors.MotorEx;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.TrapezoidProfile;

public class SpecimenLift extends SubsystemBase {
    private final MotorEx lift;
    private final Motor.Encoder encoder;
    public int current_target;
    private Telemetry telemetry;
    private final ProfiledPIDController pid;
    private final TouchSensor touchSensor;  // Touch sensor Object

    public SpecimenLift(HardwareMap hMap, Telemetry t_telemetry) {
        telemetry = t_telemetry;
        lift = new MotorEx(hMap, LiftConstants.motor_name, Motor.GoBILDA.RPM_435);
        touchSensor = hMap.get(TouchSensor.class, LiftConstants.touch_sensor_name);
        lift.setInverted(true);
        encoder = lift.encoder;
        lift.stopAndResetEncoder();
        current_target = get_position();
        lift.setRunMode(Motor.RunMode.RawPower);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                LiftConstants.maxVelocity,
                LiftConstants.maxAcceleration);
        pid = new ProfiledPIDController(LiftConstants.kP, 0., 0., constraints);
        pid.setTolerance(LiftConstants.threshold);
    }

    public int get_position() {
        return encoder.getPosition();
    }

    public boolean atTarget() {
        return pid.atGoal();
    }

    public boolean atBottom() {
        return touchSensor.isPressed();
    }

    public void runToPosition(int target) {
        current_target = safeLimit(target);
        pid.setGoal(current_target);
    }

    public int safeLimit(int target) {
        return Math.min(target, LiftConstants.full_out);
    }

    @Override
    public void periodic() {
        if (atBottom()) { encoder.reset(); }
        int newSetpoint = safeLimit(current_target);
        pid.setGoal(newSetpoint);
        double power = pid.calculate(get_position());
        lift.set(power);
        telemetry.addLine(String.format("lift enc %d tgt %d power %f touch %b",
                encoder.getPosition(),newSetpoint,power,atBottom()));
        //telemetry.update();
    }

    public void stop() {
        lift.set(0.);
    }
}
