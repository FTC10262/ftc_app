// Copyright c() 2017 FTC Team 10262 Pioπeers

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * ArcadeDrive Mode
 * <p>
 */
@TeleOp(name="Teleop 11769", group="FTC 11769")
public class Teleop11769 extends Base11769 {
    private static final int TRAY_RAMP_DURATION = 500;
    private static final double TRAY_EPSILON = 0.05;
    private static final double KICKER_RETRACTED = 0;
    private static final double KICKER_DEPLOYED = 1.0;

    private final double TRAY_DEPLOY_POSITION = 0.9;
    private final double TRAY_DRIVE_POSTION = 0.7;
    private final double TRAY_COLLECT_POSITION = 0.35;

    private TimeRampValue trayServoRamp;
    private TimeRampValue kickerServoRamp;

    public Teleop11769() {}

    @Override
    public void init() {
        super.init();
        trayServoRamp = new TimeRampValue(left_tray_servo.getPosition(), TRAY_COLLECT_POSITION, 1, new Rampable() {
            @Override
            public void tick(double value) {
                left_tray_servo.setPosition(value);
                right_tray_servo.setPosition(value);
            }
        });
        trayServoRamp.loop();

        kickerServoRamp = new TimeRampValue(KICKER_RETRACTED, KICKER_RETRACTED, 1, new Rampable() {
            @Override
            public void tick(double value) {
                kicker.setPosition(value);
            }
        });
        kickerServoRamp.loop();

//        jewel_arm.setPosition(JEWEL_ARM_RETRACTED);

        trayServoRamp.reset(TRAY_COLLECT_POSITION, TRAY_COLLECT_POSITION, 1);
        left_tray_servo.setPosition(TRAY_COLLECT_POSITION);
        right_tray_servo.setPosition(TRAY_COLLECT_POSITION);

        kickerServoRamp.reset(KICKER_RETRACTED, KICKER_RETRACTED, 1);
        kicker.setPosition(KICKER_RETRACTED);
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
        glyph_kicker_loop();
    }

    static boolean prev_kicker = false;
    protected void glyph_kicker_loop() {
        if (gamepad2.a && !prev_kicker) {
            kickerServoRamp.reset(kicker.getPosition(), KICKER_DEPLOYED, 250);
        } else if (gamepad2.back) {
            kickerServoRamp.reset(kicker.getPosition(), KICKER_RETRACTED, 50);
        }
        prev_kicker = gamepad2.a;

        kickerServoRamp.loop();
        telemetry.addData("kicker:", kicker.getPosition());
    }

    protected boolean tray_deployed() {
        return Math.abs(left_tray_servo.getPosition() - TRAY_DEPLOY_POSITION) < TRAY_EPSILON;
    }
    
    protected void tray_loop() {
        if (gamepad2.dpad_up) {
            trayServoRamp.reset(left_tray_servo.getPosition(), TRAY_DEPLOY_POSITION, TRAY_RAMP_DURATION * 4);
            kickerServoRamp.reset(kicker.getPosition(), KICKER_RETRACTED, 50);
        } else if (gamepad2.dpad_down) {
            trayServoRamp.reset(left_tray_servo.getPosition(), TRAY_COLLECT_POSITION, TRAY_RAMP_DURATION);
            kickerServoRamp.reset(kicker.getPosition(), KICKER_RETRACTED, 50);
        } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
            trayServoRamp.reset(left_tray_servo.getPosition(), TRAY_DRIVE_POSTION, TRAY_RAMP_DURATION);
            kickerServoRamp.reset(kicker.getPosition(), KICKER_RETRACTED, 50);
        }

        telemetry.addData("tray: ", left_tray_servo.getPosition());
        trayServoRamp.loop();
    }

    protected void elevator_loop() {
        elevator.setPower(gamepad2.left_stick_y);
    }

    protected void collector_loop() {
        double power = gamepad2.left_trigger - gamepad2.right_trigger;
        intake.setPower(power);
        telemetry.addData("intake: ", ""+power);
    }

}
