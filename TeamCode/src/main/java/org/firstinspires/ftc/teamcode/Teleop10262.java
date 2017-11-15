// Copyright (c) 2017 FTC Team 10262 Pioπeers

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * ArcadeDrive Mode
 * <p>
 */
@TeleOp(name="Teleop10262", group="Teleop")
public class Teleop10262 extends Base10262 {

    private int counter = 0;
    private long last = 0;
    private long total = 0;
    private ElapsedTime tray_timer;
    private boolean in_wiggle;

    enum WiggleState {
        OPEN,
        CLOSE,
        SHIFT_LEFT,
        SHIFT_RIGHT,
        CLOSED;

        private static WiggleState[] vals = values();
        public WiggleState next()
        {
            return vals[(this.ordinal()+1) % vals.length];
        }
    };

    ElapsedTime wiggle_timer;
    WiggleState wiggle_state = WiggleState.OPEN;

    protected TimeRampValue tray_ramp;

    public Teleop10262() {}

    @Override
    public void init() {
        super.init();

        wiggle_timer = new ElapsedTime();
        tray_timer = new ElapsedTime();
        last = System.nanoTime();

        tray_ramp = new TimeRampValue(Calibration10262.TRAY_COLLECT_POSITION, Calibration10262.TRAY_COLLECT_POSITION, 1,
                new Rampable() {
                    @Override
                    public void tick(double value) {
                        set_tray_angle(value);
                    }
                });

        tray_state = TrayState.COLLECTING;
        in_wiggle = false;
    }

    @Override
    public void init_loop() {
        super.init_loop();

        // Do these in init_loop so we can see the adjustments
        set_tray_angle(Calibration10262.TRAY_COLLECT_POSITION);
        open_tray();
        jewel_arm.setPosition(Calibration10262.JEWEL_ARM_RETRACTED);
    }

    @Override
    public void loop() {
        arcadeDrive(
                gamepad1.right_trigger + gamepad1.left_trigger * -1,
                 -gamepad1.left_stick_x,
                true);

        collector_loop();
        elevator_loop();
        tray_loop();
        pinch_loop();
        color_sensor_loop();
        jewel_arm_loop();
        // timing_info_loop();
    }

    private void jewel_arm_loop() {
        if (gamepad2.back) {
            jewel_arm.setPosition(Calibration10262.JEWEL_ARM_DEPLOYED);
        } else {
            jewel_arm.setPosition(Calibration10262.JEWEL_ARM_RETRACTED);
        }
    }

    protected void timing_info_loop() {
        if (counter++ % Calibration10262.SAMPLES == 0) {
            telemetry.addData("loop time: ", total / Calibration10262.SAMPLES / 1000.0);
            /*
            org.firstinspires.ftc.robotcore.external.navigation.Position pos = imu.getPosition();
            Orientation angles = imu.getAngularOrientation();
            telemetry.addData("imu: ", "x: " + angles.firstAngle + ", y: " +
                    angles.secondAngle + ", z: " + angles.thirdAngle);
                    */
            counter = 0;
            total = 0;
        }
        final long now = System.nanoTime();
        final long loop_time = now - last;
        total += loop_time;
        last = now;
    }

    protected void color_sensor_loop() {
        // probably just want an "up" command
//        telemetry.addData("Color: ", "" + jewel_color.red() + ", " + jewel_color.green() + ", " + jewel_color.blue());
        String color = "blue";
        if (jewel_color.red() > jewel_color.blue()) {
            color = "red";
        }
        telemetry.addData("Color: ", color);
    }

    protected void tray_loop() {
        switch (tray_state) {
            case DEPLOYED:
                if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) {
                    close_tray();
                    tray_state = TrayState.TO_DRIVE;
                }
                break;

            case DRIVING:
                if (gamepad2.dpad_up) {
                    close_tray();
                    tray_state = TrayState.TO_DEPLOY;
                } else if (gamepad2.dpad_down) {
                    close_tray();
                    tray_state = TrayState.TO_COLLECT;
                }
                break;

            case COLLECTING:
                if (gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right) {
                    close_tray();
                    tray_state = TrayState.TO_DRIVE;
                    tray_timer.reset();
                }
                break;

            case LEVER_UP:
                if (tray_timer.seconds() > 1) {
                    right_elevator.setPower(0);
                    left_elevator.setPower(0);
                    close_tray();
                    tray_state = TrayState.TO_DRIVE;
                } else {
                    right_elevator.setPower(-0.2);
                    left_elevator.setPower(-0.2);
                }
                break;

            case TO_DRIVE:
                if (tray_timer.seconds() > 0.5) {
                    tray_ramp.reset(tray_position(), Calibration10262.TRAY_DRIVE_POSTION, Calibration10262.TRAY_RAMP_DURATION);
                    tray_state = TrayState.DRIVING;
                }
                break;

            case TO_DEPLOY:
                tray_ramp.reset(tray_position(), Calibration10262.TRAY_DEPLOY_POSITION, Calibration10262.TRAY_RAMP_DURATION);
                tray_state = TrayState.DEPLOYED;
                break;

            case TO_COLLECT:
                tray_ramp.reset(tray_position(), Calibration10262.TRAY_COLLECT_POSITION, Calibration10262.TRAY_RAMP_DURATION);
                tray_ramp.setAtFinish(new Runnable() {
                    @Override
                    public void run() {
                        open_tray();
                    }
                });
                tray_state = TrayState.COLLECTING;
                break;
        }
        tray_ramp.loop();

        telemetry.addData("pinch:", "" + left_pinch.getPosition() + " / " + right_pinch.getPosition());
        telemetry.addData("tray: ", tray_position());
    }

    protected void pinch_loop() {
        if (gamepad2.y) {
            // wiggle states
            in_wiggle = true;
            switch (wiggle_state) {
                case OPEN:
                    wide_open_tray();
                    break;

                case CLOSE:
                    close_tray();
                    break;

                case SHIFT_LEFT:
                    left_pinch.setPosition(Calibration10262.TRAY_PINCH_CLOSE);
                    right_pinch.setPosition(Calibration10262.TRAY_PINCH_OPEN);
                    break;

                case SHIFT_RIGHT:
                    left_pinch.setPosition(Calibration10262.TRAY_PINCH_OPEN);
                    right_pinch.setPosition(Calibration10262.TRAY_PINCH_CLOSE);
                    break;

                case CLOSED:
                    close_tray();
                    break;
            }

            if (wiggle_timer.seconds() > Calibration10262.WIGGLE_WAIT) {
                wiggle_state = wiggle_state.next();
                wiggle_timer.reset();
            }
        } else if (in_wiggle) {
            in_wiggle = false;
            open_tray();
        } else if (gamepad2.a) {
            if (tray_deployed()) {
                wide_open_tray();
            } else {
                open_tray();
            }
        } else if (gamepad2.x) {
            close_tray();
//        } else if (tray_deployed()) {
//            set_tray_pinch(TRAY_PINCH_CLOSE - Math.abs(gamepad2.right_stick_x));
//        } else if (tray_collecting()) {
//            set_tray_pinch(Math.abs(gamepad2.right_stick_x));
        }
    }

    protected void elevator_loop() {
        double elevator_power = gamepad2.left_stick_y;
        if (gamepad2.start) {
            elevator_power = gamepad2.left_stick_x * Calibration10262.MAX_ELEVATOR_ADJUST;
            right_elevator.setPower(-elevator_power);
            left_elevator.setPower(elevator_power);
        } else if (Math.abs(elevator_power) > Calibration10262.ELEVATOR_DEADZONE) {
            right_elevator.setPower(-elevator_power);
            left_elevator.setPower(-elevator_power);
        } else {
            right_elevator.setPower(0);
            left_elevator.setPower(0);
        }
    }

    protected void collector_loop() {
        double reverse = 1;

        if (gamepad2.b) {
            reverse = -1;
        }

        if (gamepad2.left_bumper) {
            left_intake.setPower(-Calibration10262.MAX_INTAKE_POWER * reverse);
            right_intake.setPower(Calibration10262.MAX_INTAKE_POWER * reverse);
        } else if (gamepad2.right_bumper) {
            left_intake.setPower(Calibration10262.REVERSE_POWER * reverse);
            right_intake.setPower(-Calibration10262.REVERSE_POWER * reverse);
        } else if (in_wiggle) {
            left_intake.setPower(-Calibration10262.SLOW_COLLECT);
            right_intake.setPower(Calibration10262.SLOW_COLLECT);
        } else {
            left_intake.setPower(gamepad2.left_trigger * -Calibration10262.MAX_INTAKE_POWER * reverse);
            right_intake.setPower(gamepad2.right_trigger * Calibration10262.MAX_INTAKE_POWER * reverse);
        }
    }

}
