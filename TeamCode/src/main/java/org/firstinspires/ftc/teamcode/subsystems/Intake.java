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

    public DcMotor intake;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        intake = motorInit(opMode.hardwareMap, "intake", DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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

    public void telemetry() {
        opMode.telemetry.addData("Intake Moving", intake.getPower() != 0);
    @Override
    public String[] getTelemetry() {
        return new String[] {
                "Intake Power: " + intake.getPower(),
        };
    }
}
