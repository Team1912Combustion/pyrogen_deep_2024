
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.trajectory.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.robot.Constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.robot.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;

public class Elevator extends SubsystemBase {
    private final MotorEx elevator;
    private final Motor.Encoder encoder;
    public int current_target;
    private Arm arm;
    private Telemetry telemetry;
    private final ProfiledPIDController pid;
    private final TouchSensor touchSensor;  // Touch sensor Object

    public Elevator(HardwareMap hMap, Arm a_arm, Telemetry t_telemetry) {
        telemetry = t_telemetry;
        arm = a_arm;
        touchSensor = hMap.get(TouchSensor.class, ElevatorConstants.touch_sensor_name);
        elevator = new MotorEx(hMap, ElevatorConstants.motor_name, Motor.GoBILDA.RPM_312);
        elevator.setInverted(true);
        encoder = elevator.encoder;
        elevator.stopAndResetEncoder();
        current_target = get_position();
        elevator.setRunMode(Motor.RunMode.RawPower);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity,
                ElevatorConstants.maxAcceleration);
        pid = new ProfiledPIDController(ElevatorConstants.kP, 0., 0., constraints);
        pid.setTolerance(ElevatorConstants.threshold);
    }

    public double get_fraction() {
        double range =  ElevatorConstants.full_out - ElevatorConstants.full_in;
        return (get_position() - ElevatorConstants.full_in) / range;
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

    private int fix_target(int target) {
        return Math.min(
                ElevatorConstants.full_out,
                Math.max(ElevatorConstants.full_in, target)
        );
    }

    public void runToPosition(int target) {
        int currentTarget = fix_target(target);
        current_target = currentTarget;
        pid.setGoal(current_target);
    }

    public int safeLimit() {
        if (arm.get_position() < ArmConstants.pos_check) {
            return 1500 + arm.get_position();
        }
        return ElevatorConstants.full_out;
    }

    public int limitRange(int target) {
        return Math.min(target, safeLimit());
    }

    @Override
    public void periodic() {
        if (atBottom()) { encoder.reset(); }
        int newSetpoint = limitRange(current_target);
        pid.setGoal(newSetpoint);
        double power = pid.calculate(get_position());
        elevator.set(power);
        telemetry.addLine(String.format("elev enc %d tgt %d power %f touch %b\n",
                encoder.getPosition(),newSetpoint,power,atBottom()));
        telemetry.update();
    }

    public void stop() {
        elevator.set(0.);
    }
}
