package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.CRServoInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import java.util.List;

public class Transfer implements Subsystem {
    private LinearOpMode opMode;
    public CRServo transfer;

    public Transfer(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        transfer = CRServoInit(opMode.hardwareMap, "transfer", CRServo.Direction.FORWARD);
    }

    @Override
    public void update() {
        return;
    }

    public void start() {
        transfer.setPower(1);
    }

    @Override
    public void stop() {
        transfer.setPower(0);
    }

    @Override
    public List<String> getTelemetry() {
        return List.of(
                "Transfer State: " + (transfer.getPower() > 0 ? "on" : "off")
        );
    }
}
