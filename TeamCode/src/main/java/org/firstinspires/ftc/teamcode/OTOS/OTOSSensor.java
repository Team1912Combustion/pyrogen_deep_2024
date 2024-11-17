package org.firstinspires.ftc.teamcode.OTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;

public class OTOSSensor extends SparkFunOTOS {
    public OTOSSensor(I2cDeviceSynch deviceClient)
    {
        super(deviceClient);
    }

    public Pose2d getPose2d() {
        Pose2D sensor_pos = getPosition();
        return new Pose2d(sensor_pos.x, sensor_pos.y, Rotation2d.fromDegrees(sensor_pos.h));
    }

}
