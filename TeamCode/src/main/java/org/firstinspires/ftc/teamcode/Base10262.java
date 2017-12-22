// Copyright (c) 2017 FTC Team 10262 Pioπeers

package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Base code common to all other 10262 opmode
 */
@Disabled()
public class Base10262 extends OpMode {
    private static Context appContext;

    // Make private to ensure that our set_power routine
    // is used and power ramping is implemented
    private static DcMotor left_drive = null;
    private static DcMotor right_drive = null;

    protected static DcMotor left_intake = null;
    protected static DcMotor right_intake = null;

    protected static DcMotor left_elevator = null;
    protected static DcMotor right_elevator = null;

    protected static Servo left_tray_servo = null;
    protected static Servo right_tray_servo = null;

    protected static Servo left_pinch = null;
    protected static Servo right_pinch = null;

    protected static Servo jewel_arm = null;
    protected static ColorSensor jewel_color = null;
    protected static Servo jewel_kicker = null;

    private double maxSpeed = 1;

    protected enum TrayState {
        DEPLOYED, DRIVING, COLLECTING, TO_DRIVE,
        TO_DEPLOY, TO_COLLECT, LEVER_UP
    }
    protected TrayState tray_state = TrayState.COLLECTING;

    /**
     * Constructor
     */
    public Base10262() {
        // most if not all of your setup code
        // belongs in init, not here (see below)
    }

     /*
      * Code to run when the op mode is initialized goes here
	  */
    @Override
    public void init() {
        appContext = hardwareMap.appContext;
        new Calibration10262().readFromFile();

        left_drive = hardwareMap.dcMotor.get("left drive");
        right_drive = hardwareMap.dcMotor.get("right drive");
        // Do we want brake mode here?
        DcMotor.ZeroPowerBehavior drive_mode = DcMotor.ZeroPowerBehavior.FLOAT;
        if (Calibration10262.LOCK_DRIVE_WHEELS) {
            drive_mode = DcMotor.ZeroPowerBehavior.BRAKE;
        }
        left_drive.setZeroPowerBehavior(drive_mode);
        right_drive.setZeroPowerBehavior(drive_mode);

        DcMotor.ZeroPowerBehavior intake_mode = DcMotor.ZeroPowerBehavior.FLOAT;
        if (Calibration10262.LOCK_INTAKE_WHEELS) {
            intake_mode = DcMotor.ZeroPowerBehavior.BRAKE;
        }
        left_intake = hardwareMap.dcMotor.get("left intake");
        right_intake = hardwareMap.dcMotor.get("right intake");
        left_intake.setZeroPowerBehavior(intake_mode);
        right_intake.setZeroPowerBehavior(intake_mode);

        left_elevator = hardwareMap.dcMotor.get("left elevator");
        right_elevator = hardwareMap.dcMotor.get("right elevator");
        left_elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_tray_servo = hardwareMap.servo.get("left tray");
        left_tray_servo.setDirection(Servo.Direction.REVERSE);
        right_tray_servo = hardwareMap.servo.get("right tray");
        right_tray_servo.setDirection(Servo.Direction.FORWARD);

        left_pinch = hardwareMap.servo.get("left pinch");
        left_pinch.setDirection(Servo.Direction.FORWARD);
        right_pinch = hardwareMap.servo.get("right pinch");
        right_pinch.setDirection(Servo.Direction.REVERSE);

        jewel_arm = hardwareMap.servo.get("jewel arm");
        jewel_kicker = hardwareMap.servo.get("jewel kicker");
        jewel_color = hardwareMap.get(ColorSensor.class, "jewel color");

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        // do nothing
    }

    static Context getContext() {
        return appContext;
    }

    /*
	 * Code to run when the op mode is first disabled goes here
	 *
	 */
    @Override
    public void stop() {
        set_drive_power(0,0);
        stop_intake();
        stop_elevator();
    }

    public void stop_all_motors() {
        stop_intake();
        stop_elevator();
        set_drive_power(0,0);
    }

    private void stop_intake() {
        left_intake.setPower(0);
        right_intake.setPower(0);
    }

    protected void stop_elevator() {
        left_elevator.setPower(0);
        right_elevator.setPower(0);
    }

    protected void set_tray_angle(double pos) {
        left_tray_servo.setPosition(pos);
        right_tray_servo.setPosition(pos);
    }

    protected void open_tray() {
        left_pinch.setPosition(Calibration10262.TRAY_PINCH_OPEN_LEFT);
        right_pinch.setPosition(Calibration10262.TRAY_PINCH_OPEN_RIGHT);
    }

    protected void wide_open_tray() {
        left_pinch.setPosition(Calibration10262.TRAY_PINCH_WIDE_OPEN_LEFT);
        right_pinch.setPosition(Calibration10262.TRAY_PINCH_WIDE_OPEN_RIGHT);
    }

    private double tray_pinch() {
        return (left_pinch.getPosition() + right_pinch.getPosition()) / 2;
    }

    protected boolean tray_open() {
        if (Math.abs(left_pinch.getPosition() - Calibration10262.TRAY_PINCH_OPEN_LEFT) < Calibration10262.TRAY_PINCH_EPSILON) {
            if (Math.abs(right_pinch.getPosition() - Calibration10262.TRAY_PINCH_OPEN_RIGHT) < Calibration10262.TRAY_PINCH_EPSILON) {
                return true;
            }
        }
        return false;
    }

    protected void close_tray() {
        left_pinch.setPosition(Calibration10262.TRAY_PINCH_CLOSE_LEFT);
        right_pinch.setPosition(Calibration10262.TRAY_PINCH_CLOSE_RIGHT);
    }

    protected double tray_position() {
        return (left_tray_servo.getPosition() + right_tray_servo.getPosition()) / 2;
    }

    protected boolean tray_deployed() {
        return Math.abs(tray_position() - Calibration10262.TRAY_DEPLOY_POSITION) < Calibration10262.TRAY_EPSILON;
    }

    protected boolean tray_collecting() {
        return Math.abs(tray_position() - Calibration10262.TRAY_COLLECT_POSITION) < Calibration10262.TRAY_EPSILON;
    }

    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    protected static double limit(double num) {
        return limit(-1, 1, num);
    }

    protected static double limit(double min, double max, double num) {
        if (num > max) {
            return max;
        }
        if (num < min) {
            return min;
        }

        return num;
    }

    protected void setMaxSpeed(double val) {
        maxSpeed = val;
    }

    protected double getMaxSpeed() {
        return maxSpeed;
    }

    protected static double ramp(double oldVal, double newVal, double maxRamp) {
        double delta = newVal - oldVal;
        if (delta > maxRamp) {
            delta = maxRamp;
        } else if (delta < -maxRamp) {
            delta = -maxRamp;
        }
        return oldVal + delta;
    }

    private double prevLeftPower = 0;
    private double prevRightPower = 0;
    private long lastSetDrivePower = 0;
    protected void set_drive_power(double left, double right) {
        telemetry.addData("DRIVE: ", "" + left + "," + right);
//        if (Calibration10262.RAMP_DRIVE_POWER) {
//            final double maxChangePerMilliSecond = Calibration10262.RAMP_DRIVE_DURATION;
//            final long ticks = System.currentTimeMillis() - lastSetDrivePower;
//            final double maxRamp = Math.max(1, maxChangePerMilliSecond * ticks);
//
//            left = ramp(prevLeftPower, left, maxRamp);
//            right = ramp(prevRightPower, right, maxRamp);
//            prevLeftPower = left;
//            prevRightPower = right;
//        }
        left = limit(-maxSpeed, maxSpeed, left);
        right = limit(-maxSpeed, maxSpeed, right);
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

        moveValue = limit(moveValue, maxSpeed);
        rotateValue = limit(rotateValue, maxSpeed);

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
//            moveValue = moveValue * moveValue * moveValue;
//            rotateValue = rotateValue * rotateValue * rotateValue;
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

    private double limit(double maxSpeed, double val) {
        return limit(-maxSpeed, maxSpeed, val);
    }

}
