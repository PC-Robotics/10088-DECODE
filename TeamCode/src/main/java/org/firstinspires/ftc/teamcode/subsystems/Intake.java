package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Locale;
import java.util.stream.Stream;

@Configurable
public class Intake implements Subsystem {
    private LinearOpMode opMode;
    public DcMotorEx intake;
    public DistanceSensor distanceSensor;

    private final boolean usingDistanceSensor = false;
    public double distance = 0.0; // in

    public boolean detectingBall = false;
    public static double BALL_DETECTING_DISTANCE = 7.0;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        intake = motorInit(opMode.hardwareMap, "intake", DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        if (usingDistanceSensor) {
            distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");
        }
    }


    @Override
    public void update() {
        if (!usingDistanceSensor) {
            detectingBall = false;
            return;
        }

        distance = distanceSensor.getDistance(DistanceUnit.INCH);
        detectingBall = distance < BALL_DETECTING_DISTANCE;
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
        if (usingDistanceSensor && distance != 0.0) {
            colorArr = new String[]{
                    "Intake Distance: " + String.format(Locale.ROOT,"%.01f in", distance),
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
}
