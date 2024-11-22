package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.pyrolib.OTOS.OTOSSensor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OdoTestCommand extends CommandOpMode {

    private Drive m_drive;
    private DefaultDrive m_driveCommand;
    private GamepadEx m_driverStick;

    public void initialize() {

        m_driverStick = new GamepadEx(gamepad1);

        // get our OTOS sensor
        OTOSSensor m_OTOS = hardwareMap.get(OTOSSensor.class, "sensor_otos");

        // create our drive object
        m_drive = new Drive(hardwareMap, "front_left", "front_right",
                "back_left", "back_right", m_OTOS);
        register(m_drive);
        m_driveCommand = new DefaultDrive(m_drive,
                () -> m_driverStick.getLeftX(),
                () -> m_driverStick.getLeftY(),
                () -> m_driverStick.getRightX());
        m_drive.setDefaultCommand(m_driveCommand);
    }

}
