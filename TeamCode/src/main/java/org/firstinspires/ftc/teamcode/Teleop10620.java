// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * ArcadeDrive Mode
 * <p>
 */
@TeleOp(name="Teleop 10620", group="FTC 10620")
public class Teleop10620 extends Base10620 {
    private static final double ELEVATOR_DEADZONE = 0.1;
    private static final double MAX_ELEVATOR_ADJUST = 0.2;
    private static final double REVERSE_POWER = 0.4;
    private static final double MAX_INTAKE_POWER = 1.0;

    protected static final double TRAY_DEPLOY_POSITION = 0.55;
    protected static final double TRAY_DRIVE_POSTION = 0.3;
    protected static final double TRAY_COLLECT_POSITION = 0;

    protected static final double KICKER_OUT = 0.9;
    protected static final double KICKER_IN = 0.1;

    /**
     * Constructor
     */
    public Teleop10620() {
        // most if not all of your setup code
        // belongs in init, not here (see below)
    }

    /*
     * Code to run when the op mode is initialized goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
	 */
    @Override
    public void init() {
        super.init();
        jewel_arm.setPosition(JEWEL_ARM_RETRACTED);
    }

    /*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
    @Override
    public void loop() {
        arcadeDrive(gamepad1.left_trigger - gamepad1.right_trigger, gamepad1.left_stick_x, true);

        collector_loop();
        tray_loop();
        elevator_loop();

//        kicker_loop();
    }

    private void collector_loop() {
        double reverse = 1;

        if (gamepad2.b) {
            reverse = -1;
        }

        if (gamepad2.left_bumper) {
            left_intake.setPower(-MAX_INTAKE_POWER * reverse);
            right_intake.setPower(MAX_INTAKE_POWER * reverse);
        } else if (gamepad2.right_bumper) {
            left_intake.setPower(REVERSE_POWER * reverse);
            right_intake.setPower(-REVERSE_POWER * reverse);
        } else {
            left_intake.setPower(gamepad2.left_trigger * -MAX_INTAKE_POWER * reverse);
            right_intake.setPower(gamepad2.right_trigger * MAX_INTAKE_POWER * reverse);
        }
    }

    private void tray_loop() {
        if (gamepad2.dpad_up) {
            set_tray_angle(TRAY_DEPLOY_POSITION);
        } else if (gamepad2.dpad_down) {
            set_tray_angle(TRAY_COLLECT_POSITION);
        } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
            set_tray_angle(TRAY_DRIVE_POSTION);
        }
    }

    private void elevator_loop() {
        double elevator_power = gamepad2.left_stick_y;
        if (gamepad2.start) {
            elevator_power = gamepad2.left_stick_x * MAX_ELEVATOR_ADJUST;
            right_elevator.setPower(-elevator_power);
            left_elevator.setPower(elevator_power);
        } else if (Math.abs(elevator_power) > ELEVATOR_DEADZONE) {
            right_elevator.setPower(-elevator_power);
            left_elevator.setPower(-elevator_power);
        } else {
            right_elevator.setPower(0);
            left_elevator.setPower(0);
        }
    }

//    private void kicker_loop() {
//        if (gamepad2.a) {
//            kicker.setPosition(KICKER_OUT);
//        } else {
//            kicker.setPosition(KICKER_IN);
//        }
//    }

    private boolean ready_to_deploy() {
        return Math.abs(left_tray_servo.getPosition() - TRAY_DEPLOY_POSITION) < 0.1;
    }

}
