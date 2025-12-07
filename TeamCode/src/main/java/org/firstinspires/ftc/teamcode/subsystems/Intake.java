package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.CRServoInit;
import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;

@Configurable
public class Intake implements Subsystem {
    private LinearOpMode opMode;
    public DcMotorEx intake;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        intake = motorInit(
                opMode.hardwareMap,
                "intake",
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void update() {
        return;
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
    public List<String> getTelemetry() {
        return List.of(
            "Intake Power: " + intake.getPower()
        );
    }
}
