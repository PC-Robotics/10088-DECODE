package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class OurIMU {
    private LinearOpMode opMode;

    public IMU imu;
    private IMU.Parameters paramters;

    public OurIMU(LinearOpMode opMode) {
        this.opMode = opMode;

        paramters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP // its backwards
                )
        );
    }

    public void init() {
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(paramters);
    }

    // get an object containing euler angles
    public YawPitchRollAngles getEulerAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    public double getHeading() {
        return getHeading(AngleUnit.RADIANS);
    }

    public double getHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void telemetry() {
        opMode.telemetry.addData("Heading", getHeading());
    }
}
