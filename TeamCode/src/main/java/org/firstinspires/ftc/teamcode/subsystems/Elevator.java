
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.controller.PController;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.controller.wpilibcontroller.ArmFeedforward;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class Elevator extends SubsystemBase {
    private final MotorEx m_elevator;
    private final Motor.Encoder m_encoder;
    public int current_target;
    private Arm m_arm;
    private Telemetry telemetry;
    private final PController pid;
    private final TouchSensor touchSensor;  // Touch sensor Object

    public Elevator(HardwareMap hMap, Arm a_arm, Telemetry t_telemetry) {
        telemetry = t_telemetry;
        m_arm = a_arm;
        touchSensor = hMap.get(TouchSensor.class, ElevatorConstants.touch_sensor_name);
        m_elevator = new MotorEx(hMap, ElevatorConstants.motor_name, Motor.GoBILDA.RPM_312);
        m_elevator.setInverted(true);
        m_encoder = m_elevator.encoder;
        m_elevator.stopAndResetEncoder();
        current_target = get_position();
        m_elevator.setRunMode(Motor.RunMode.RawPower);
        pid = new PController(Constants.ElevatorConstants.kP);
        pid.setTolerance(Constants.ElevatorConstants.threshold);
    }

    public double get_fraction() {
        double range =  ElevatorConstants.full_out - ElevatorConstants.full_in;
        return (get_position() - ElevatorConstants.full_in) / range;
    }

    public int get_position() {
        return m_encoder.getPosition();
    }

    public boolean atTarget() {
        return pid.atSetPoint();
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
        pid.setSetPoint(current_target);
    }

    public int limitRange(int target) {
        if (m_arm.get_position() < Constants.ArmConstants.pos_mid) {
            return Math.min(target, 1500 + m_arm.get_position());
        }
        return target;
    }

    @Override
    public void periodic() {
        if (atBottom()) { m_encoder.reset(); }
        int newSetpoint = limitRange(current_target);
        pid.setSetPoint(newSetpoint);
        double power = pid.calculate(get_position());
        m_elevator.set(power);
        telemetry.addLine(String.format("elev enc %d tgt %d power %f touch %b\n",
                m_encoder.getPosition(),current_target,power,atBottom()));
    }

    public void stop() {
        m_elevator.set(0.);
    }
}
