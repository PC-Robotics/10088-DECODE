package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Utility class for hardware initialization.
 */
public class HardwareUtility {
    private HardwareUtility() {
        throw new UnsupportedOperationException(
                "This is a utility class and cannot be instantiated"
        );
    }

    /**
     * Initializes a DcMotorEx with the specified parameters.
     *
     * @param hardwareMap Hardware map to access robot configuration
     * @param name        Name of the motor in the hardware map
     * @param direction   Direction for the motor
     * @return Configured DcMotorEx instance
     */
    public static DcMotorEx motorInit(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    /**
     * Initializes a Servo with the specified parameters.
     *
     * @param hardwareMap Hardware map to access robot configuration
     * @param name        Name of the servo in the hardware map
     * @param direction   Direction for the servo
     * @return Configured Servo instance
     */
    public static Servo servoInit(HardwareMap hardwareMap, String name, Servo.Direction direction) {
        Servo servo = hardwareMap.get(Servo.class, name);
        servo.setDirection(direction);
        return servo;
    }

    /**
     * Initializes a CRServo with the specified parameters.
     *
     * @param hardwareMap Hardware map to access robot configuration
     * @param name        Name of the CRServo in the hardware map
     * @param direction   Direction for the CRServo
     * @return Configured CRServo instance
     */
    public static CRServo CRServoInit(HardwareMap hardwareMap, String name, CRServo.Direction direction) {
        CRServo crServo = hardwareMap.get(CRServo.class, name);
        crServo.setDirection(direction);
        return crServo;
    }
}
