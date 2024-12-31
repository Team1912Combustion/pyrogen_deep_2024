
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.controller.PController;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.controller.wpilibcontroller.ArmFeedforward;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final MotorEx m_elevator;
    private final Motor.Encoder m_encoder;
    public int current_target;
    private Telemetry telemetry;
    private final PController pid;

    public Elevator(HardwareMap hMap, String motorName, Telemetry t_telemetry) {
        telemetry = t_telemetry;
        m_elevator = new MotorEx(hMap, motorName, Motor.GoBILDA.RPM_312);
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

    private int fix_target(int target) {
        return Math.min(
                ElevatorConstants.full_out,
                Math.max(ElevatorConstants.full_in, target)
        );
    }

    public void runToPosition(int target) {
        current_target = fix_target(target);
        pid.setSetPoint(current_target);
    }

    @Override
    public void periodic() {
        double power = pid.calculate(get_position());
        if (current_target < ElevatorConstants.threshold) {power = Math.max(power, -0.1);}
        m_elevator.set(power);
        telemetry.addLine(String.format("elev enc %d tgt %d power %f\n",
                m_encoder.getPosition(),current_target,power));
    }

    public void stop() {
        m_elevator.set(0.);
    }
}
