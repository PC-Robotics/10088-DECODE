package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.CRServoInit;
import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Settings;

import java.util.Arrays;
import java.util.stream.Stream;

public class Intake implements Subsystem {
    private LinearOpMode opMode;
    public DcMotorEx intake;
    // note that the REV Color Sensor v3 has both a color sensor and an IR proximity sensor in the same package
    public NormalizedColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    public boolean usingColorSensor = false;

    NormalizedRGBA colors;
    public float[] hsvValues = {0F, 0F, 0F};

    private static final float GREEN_HUE  = 120f;
    private static final float PURPLE_HUE = 285f;
    private static final float HUE_TOLERANCE = 25f; // +- degrees
    private static final float ALPHA_THRESHOLD = 0.05f; // minimum alpha (0–1)

    public boolean detectingBall = false;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        intake = motorInit(opMode.hardwareMap, "intake", DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        if (usingColorSensor) {
            colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
            distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "colorSensor");
        }
    }


    @Override
    public void update() {
        if (!usingColorSensor) {
            detectingBall = false;
            return;
        }

        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        detectingBall = colors.alpha > ALPHA_THRESHOLD && (
                hueDistance(hsvValues[0], GREEN_HUE) < HUE_TOLERANCE ||
                hueDistance(hsvValues[0], PURPLE_HUE) < HUE_TOLERANCE
        );
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
        String[] colorArr;
        if (usingColorSensor) {
            colorArr = new String[]{
                    "R: " + colors.red + " G: " + colors.green + " B: " + colors.blue + " A: " + colors.alpha,
                    "Ball Detected: " + (detectingBall ? "yes" : "no")
            };
        } else {
            colorArr = new String[0];
        }

        String[] normalArr = new String[] {
            "Intake Power: " + intake.getPower(),
        };

        return Stream.concat(Arrays.stream(normalArr), Arrays.stream(colorArr))
                .toArray(String[]::new);
    }


    private static float hueDistance(float h1, float h2) {
        float distance = Math.abs(h1 - h2);
        return (distance > 180) ? 360 - distance : distance;
    }
}
