package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 10620ArcadeDrive Mode
 * <p>
 */

//@TeleOp(name = "Tele 10620", group = "10620")
@Disabled
public class Tele10620 extends OpMode {
    private static DcMotor Left_Drive = null;
    private static DcMotor Right_Drive = null;
    private static DcMotor Left_Intake = null;
    private static DcMotor Right_Intake = null;
    private static DcMotor Left_Elevator = null;
    private static DcMotor Right_Elevator = null;
    private static Servo Tray_Servo1 = null;
    private static Servo Tray_Servo2 = null;

@Override
    public void init() {
        Left_Drive = hardwareMap.dcMotor.get("front left Drive Wheel");
        Right_Drive = hardwareMap.dcMotor.get("front right Dive Wheel");
        Left_Intake = hardwareMap.dcMotor.get("Left Intake Compository Wheel");
        Right_Intake = hardwareMap.dcMotor.get("Right Intake Compository Wheel");
        Left_Elevator = hardwareMap.dcMotor.get("Left Hex Motor Elevator");
        Right_Elevator = hardwareMap.dcMotor.get("Right Hex Motor Elevator");
        Tray_Servo1 = hardwareMap.servo.get("First Servo That Rotates The Tray");
        Tray_Servo2 = hardwareMap.servo.get("Second Servo That Rotates The Tray");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        telemetry.addData("Text", "***ArcadeDrive10620***");

        arcadeDrive(gamepad1.right_trigger + gamepad1.left_trigger * -1, -gamepad1.left_stick_x, true);

        Left_Intake.setPower(gamepad2.left_trigger);
        Right_Intake.setPower(gamepad2.right_trigger);
        Left_Elevator.setPower(gamepad2.left_stick_y);
        Right_Elevator.setPower(gamepad2.left_stick_y);
        if (gamepad2.a) {
        Tray_Servo1.setPosition(0.2); // drive position
        Tray_Servo2.setPosition(0.2);
        }
        if (gamepad2.b) {
        Tray_Servo1.setPosition(0); // collect position
        Tray_Servo2.setPosition(0);
        }
        if (gamepad2.y){
        Tray_Servo1.setPosition(0.5); // deploy position
        Tray_Servo2.setPosition(0.5);
        }
        if (gamepad2.x){
        Tray_Servo1.setPosition(0.45); // transport position
        Tray_Servo2.setPosition(0.45);
        }
    }

    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    public void set_power(double left, double right) {
        Left_Drive.setPower(+left);
        Right_Drive.setPower(-right);
        Left_Intake.setPower(+left);
        Right_Intake.setPower(-right);
        Left_Elevator.setPower(+left);
        Right_Elevator.setPower(+right);
        Left_Elevator.setPower(-left);
        Right_Elevator.setPower(-right);
    }

    public void drive(double speed) {
        set_power(speed, speed);

    }

    /**
     * Arcade drive implements single stick driving. This function lets you
     * directly provide joystick values from any source.
     * $
     *
     * @param moveValue     The value to use for forwards/backwards
     * @param rotateValue   The value to use for the rotate right/left
     * @param squaredInputs If set, decreases the sensitivity at low speeds
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control
            // while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        set_power(leftMotorSpeed, rightMotorSpeed);
    }
}
