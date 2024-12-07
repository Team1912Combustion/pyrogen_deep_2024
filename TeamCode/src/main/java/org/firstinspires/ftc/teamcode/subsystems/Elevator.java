
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final MotorEx m_elevator;
    private final Motor.Encoder m_encoder;
    private int current_target;
    private Telemetry m_telemetry;

    public Elevator(HardwareMap hMap, String motorName, Telemetry telemetry) {
        m_elevator = new MotorEx(hMap, motorName);
        m_elevator.setInverted(true);
        m_encoder = m_elevator.encoder;

        m_elevator.stopAndResetEncoder();
        m_elevator.setRunMode(Motor.RunMode.VelocityControl);
        m_telemetry = telemetry;
    }

    public int get_position() {
        return m_encoder.getPosition();
    }

    public boolean allIn() {
        return (m_encoder.getPosition() < ElevatorConstants.threshold);
    }
    public boolean allOut() {
        return (ElevatorConstants.full_out - m_encoder.getPosition() < ElevatorConstants.threshold);
    }

    public boolean atTarget() {
        return (Math.abs(m_encoder.getPosition() - current_target) < ElevatorConstants.threshold);
    }

    public boolean safeToMove() {
        return (! allIn() && ! allOut());
    }

    private int fix_target(int target) {
        return Math.min(
                ElevatorConstants.full_out,
                Math.max(ElevatorConstants.full_in, target)
        );
    }

    public void runToPosition(int target) {
        current_target = fix_target(target);
        m_elevator.setRunMode(Motor.RunMode.PositionControl);
        m_elevator.setTargetPosition(current_target);
        m_elevator.set(ElevatorConstants.run_speed);
        m_telemetry.addData("elev enc:",m_encoder.getPosition());
    }

    public void move(double speed) {
        m_elevator.setRunMode(Motor.RunMode.VelocityControl);
        m_elevator.set(speed);
        m_telemetry.addData("elev enc:",m_encoder.getPosition());
    }

    public int getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_elevator.set(0.);
    }
}
