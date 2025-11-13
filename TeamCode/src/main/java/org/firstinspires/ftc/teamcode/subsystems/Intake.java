package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.CRServoInit;
import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Settings;

public class Intake implements Subsystem {
    private LinearOpMode opMode;
    public DcMotorEx intake;
    // note that the REV Color Sensor v3 has both a color sensor and an IR proximity sensor in the same package
    public NormalizedColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    NormalizedRGBA colors;
    public float[] hsvValues = {0F, 0F, 0F};

    public DcMotor intake;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        intake = motorInit(opMode.hardwareMap, "intake", DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "colorSensor");
    }

    public void intake() {
        intake.setPower(1);
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    public void outtake() {
        intake.setPower(-0.3);
    }

    public void outtake(double power) {
        intake.setPower(-power);
    }

    @Override
    public void stop() {
        intake.setPower(0);
    }

    @Override
    public String[] getTelemetry() {
        return new String[] {
                "Intake Power: " + intake.getPower(),
                "R: " + colors.red + " G: " + colors.green + " B: " + colors.blue + " A: " + colors.alpha,
                "Ball Detected: " + (detectingBall ? "yes" : "no")
        };
    }
}
