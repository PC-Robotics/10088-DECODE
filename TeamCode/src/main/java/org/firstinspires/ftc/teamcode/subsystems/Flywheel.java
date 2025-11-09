package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Flywheel {
    public enum FLYWHEEL_STATE {
        IDLE,
        SPINNING
    }

    private LinearOpMode opMode;

    public DcMotor flywheelLeft;
    public DcMotor flywheelRight;

    public FLYWHEEL_STATE state;

    public Flywheel(LinearOpMode opMode) {
        this.opMode = opMode;
        state = FLYWHEEL_STATE.IDLE;
    }

    public void init() {
        flywheelLeft = motorInit(opMode.hardwareMap, "flywheelleft", DcMotorSimple.Direction.REVERSE);
        flywheelRight = motorInit(opMode.hardwareMap, "flywheelright", DcMotorSimple.Direction.FORWARD);

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void spinToSpeed(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
        state = FLYWHEEL_STATE.SPINNING;
    }

    public void spinToSpeed() {
        spinToSpeed(.65);
    }

    public void stop() {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
        state = FLYWHEEL_STATE.IDLE;
    }
}
