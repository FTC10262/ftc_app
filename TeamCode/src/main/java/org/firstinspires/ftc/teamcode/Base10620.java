// Copyright (c) 2017 FTC Team 10262 Pioπeers

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * ArcadeDrive Mode
 * <p>
 */
@Disabled()
public abstract class Base10620 extends OpMode {
    protected static final double JEWEL_ARM_DEPLOYED = 1.0;
    protected static final double JEWEL_ARM_RETRACTED = 0.45;

    protected static DcMotor left_drive = null;
    protected static DcMotor right_drive = null;
    protected static DcMotor left_intake = null;
    protected static DcMotor right_intake = null;
    protected static DcMotor left_elevator = null;
    protected static DcMotor right_elevator = null;
    protected static Servo left_tray_servo = null;
    protected static Servo right_tray_servo = null;
//    protected static Servo kicker = null;
    protected static Servo jewel_arm = null;
    protected static ColorSensor jewel_color = null;
//    protected static BNO055IMU imu = null;

    /**
     * Constructor
     */
    public Base10620() {
        // most if not all of your setup code
        // belongs in init, not here (see below)
    }

     /* Code to run when the op mode is initialized goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
	 */
    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left drive");
        right_drive = hardwareMap.dcMotor.get("right drive");
        // Do we want brake mode here?
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_intake = hardwareMap.dcMotor.get("left intake");
        right_intake = hardwareMap.dcMotor.get("right intake");
        left_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left_elevator = hardwareMap.dcMotor.get("left elevator");
        right_elevator = hardwareMap.dcMotor.get("right elevator");
        left_elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_tray_servo = hardwareMap.servo.get("left tray");
        left_tray_servo.setDirection(Servo.Direction.REVERSE);
        right_tray_servo = hardwareMap.servo.get("right tray");
        right_tray_servo.setDirection(Servo.Direction.FORWARD);

//        kicker = hardwareMap.servo.get("kicker");
        jewel_arm = hardwareMap.servo.get("jewel arm");
        jewel_color = hardwareMap.get(ColorSensor.class, "jewel color");

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu.initialize(parameters);
    }

    /*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
    @Override
    public void stop() {
        set_drive_power(0,0);
        stop_intake();
        stop_elevator();
    }

    private void stop_intake() {
        left_intake.setPower(0);
        right_intake.setPower(0);
    }

    protected void stop_elevator() {
        left_elevator.setPower(0);
        right_elevator.setPower(0);
    }

    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    protected void set_tray_angle(double pos) {
        left_tray_servo.setPosition(pos);
        right_tray_servo.setPosition(pos);
    }

    protected void set_drive_power(double left, double right) {
        this.left_drive.setPower(-left);
        this.right_drive.setPower(right);
    }

    /**
     * Arcade drive implements single stick driving. This function lets you
     * directly provide joystick values from any source.
     *$
     * @param moveValue The value to use for forwards/backwards
     * @param rotateValue The value to use for the rotate right_drive/left_drive
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

        set_drive_power(leftMotorSpeed, rightMotorSpeed);
    }

}
