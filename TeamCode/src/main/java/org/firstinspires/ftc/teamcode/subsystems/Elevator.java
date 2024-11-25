
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants;

public class Elevator extends SubsystemBase {
    private final MotorEx m_elevator;
    private final Motor.Encoder m_encoder;

    public Elevator(HardwareMap hMap, String motorName) {
        m_elevator = new MotorEx(hMap, motorName);
        m_encoder = m_elevator.encoder;

        m_elevator.stopAndResetEncoder();
        m_elevator.setRunMode(Motor.RunMode.VelocityControl);
    }

    public int get_position() {
        return m_encoder.getPosition();
    }

    public boolean allIn() {
        return (m_encoder.getPosition() < Constants.ElevatorConstants.threshold);
    }
    public boolean allOut() {
        return (Constants.ElevatorConstants.full_out - m_encoder.getPosition() < Constants.ElevatorConstants.threshold);
    }
    public boolean safeToMove() {
        return (! allIn() && ! allOut());
    }

    public void runToPosition(int target) {
        int fix_target = Math.min(
                Constants.ElevatorConstants.full_out,
                Math.max(Constants.ElevatorConstants.full_in, target)
        );
        m_elevator.setRunMode(Motor.RunMode.PositionControl);
        m_elevator.setTargetPosition(fix_target);
    }

    public void move(double speed) {
        m_elevator.setRunMode(Motor.RunMode.VelocityControl);
        m_elevator.set(speed);
    }

    public void stop() {
        m_elevator.set(0.);
    }
}
