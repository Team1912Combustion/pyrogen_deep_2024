package org.firstinspires.ftc.teamcode.pyrolib.OTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.geometry.Rotation2d;

@I2cDeviceType
@DeviceProperties(
        name = "Extend SparkFun OTOS",
        xmlTag = "OTOSSensor",
        description = "Extend SparkFun Qwiic Optical Tracking Odometry Sensor"
)
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
