package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by phurley on 12/1/17.
 */
@TeleOp(name="Calibrate 10262", group="Teleop")
public class Calibrate extends Auton10262 {
    protected MenuController menu_controller = null;

    @Override
    public void init() {
        super.init();
        menu_controller = new MenuController(new Calibration10262());
    }

    @Override
    public void loop() {
        menu_controller.loop(telemetry, gamepad1);
        String fieldName = menu_controller.getCurrentField().getName();

//        public static double JEWEL_ARM_DEPLOYED = 0.1;
//        public static double JEWEL_ARM_RETRACTED = 0.9;

        if (fieldName.equals("JEWEL_ARM_DEPLOYED")) {
            jewel_arm.setPosition(Calibration10262.JEWEL_ARM_DEPLOYED);
        } else {
            jewel_arm.setPosition(Calibration10262.JEWEL_ARM_RETRACTED);
        }

        if (fieldName.equals("TRAY_DEPLOY_POSITION")) {
            set_tray_angle(Calibration10262.TRAY_DEPLOY_POSITION);
        } else if (fieldName.equals("TRAY_DRIVE_POSITION")) {
            set_tray_angle(Calibration10262.TRAY_DRIVE_POSITION);
        } else {
            set_tray_angle(Calibration10262.TRAY_COLLECT_POSITION);
        }

        if (fieldName.equals("TRAY_PINCH_CLOSE_LEFT") || fieldName.equals("TRAY_PINCH_CLOSE_RIGHT")) {
            close_tray();
        } else if (fieldName.equals("TRAY_PINCH_WIDE_OPEN_LEFT") || fieldName.equals("TRAY_PINCH_WIDE_OPEN_RIGHT")) {
            wide_open_tray();
        } else {
            open_tray();
        }

        if (fieldName.equals("JEWEL_KICK_FORWARD")) {
            jewel_kicker.setPosition(Calibration10262.JEWEL_KICK_FORWARD);
        } else if (fieldName.equals("JEWEL_KICK_BACK")) {
            jewel_kicker.setPosition(Calibration10262.JEWEL_KICK_BACK);
        } else {
            jewel_kicker.setPosition(Calibration10262.JEWEL_KICK_CENTER);
        }

        if (menu_controller.buttonX()) {
            if (fieldName.equals("HARD_FORWARD1_TIME") || fieldName.equals("HARD_FORWARD1_SPEED")) {
                drive_time = Calibration10262.HARD_FORWARD1_TIME;
                drive_speed = Calibration10262.HARD_FORWARD1_SPEED;
                resetState(State.RAMP_UP);
            } else if (fieldName.equals("HARD_FORWARD0_TIME") || fieldName.equals("HARD_FORWARD0_SPEED")) {
                    drive_time = Calibration10262.HARD_FORWARD0_TIME;
                    drive_speed = Calibration10262.HARD_FORWARD0_SPEED;
                    resetState(State.RAMP_UP);
            } else if (fieldName.equals("HARD_FORWARD2_TIME") || fieldName.equals("HARD_FORWARD2_SPEED")) {
                drive_time = Calibration10262.HARD_FORWARD2_TIME;
                drive_speed = Calibration10262.HARD_FORWARD2_SPEED;
                resetState(State.RAMP_UP);
            } else if (fieldName.equals("EASY_FORWARD_TIME") || fieldName.equals("EASY_FORWARD_SPEED")) {
                drive_time = Calibration10262.EASY_FORWARD_TIME;
                drive_speed = Calibration10262.EASY_FORWARD_SPEED;
                resetState(State.RAMP_UP);
            } else if (fieldName.equals("BACKOFF_SPEED") || fieldName.equals("BACKOFF_TIME")) {
                resetState(State.BACKOFF);
            } else if (fieldName.equals("ROTATION_EPSILON") || fieldName.equals("ROTATION_SPEED")) {
                resetState(State.ROTATE90);
            } else if (fieldName.equals("GLYPH_EJECT_TIME") || fieldName.equals("GLYPH_EJECT_POWER")) {
                resetState(State.PUSH_GLYPH);
            } else if (fieldName.equals("BACKOFF_SPEED") || fieldName.equals("BACKOFF_TIME")) {
                resetState(State.BACKOFF);
            } else {
                resetState(State.STOP);
            }
            exit_state = State.STOP;
        }

        if (gamepad1.x) {
            super.loop();
        } else {
            stop_all_motors();
        }
    }
}
