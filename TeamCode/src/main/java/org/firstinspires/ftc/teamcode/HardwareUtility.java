package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
     * Initializes a DcMotor with the specified parameters.
     *
     * @param hardwareMap Hardware map to access robot configuration
     * @param name        Name of the motor in the hardware map
     * @param direction   Direction for the motor
     * @return Configured DcMotor instance
     */
    public static DcMotor motorInit(HardwareMap hardwareMap, String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
