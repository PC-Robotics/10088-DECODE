package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.CRServoInit;
import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

public class Transfer implements Subsystem {
    public enum TRANSFER_STATE {
        RUNNING(1.0),
        STOPPED(-0.1);

        final double power;
        TRANSFER_STATE(double power) {
            this.power = power;
        }
    }

    private LinearOpMode opMode;
    public DcMotorEx transfer;
    public TRANSFER_STATE transferState = TRANSFER_STATE.STOPPED;

    public Transfer(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        transfer = motorInit(opMode.hardwareMap, "transfer", DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        return;
    }

    public void start() {
        transfer.setPower(TRANSFER_STATE.RUNNING.power);
        transferState = TRANSFER_STATE.RUNNING;
    }

    @Override
    public void stop() {
        transfer.setPower(TRANSFER_STATE.STOPPED.power);
        transferState = TRANSFER_STATE.STOPPED;
    }

    @Override
    public List<String> getTelemetry() {
        return List.of(
                "Transfer State: " + transferState.toString()
        );
    }
}
