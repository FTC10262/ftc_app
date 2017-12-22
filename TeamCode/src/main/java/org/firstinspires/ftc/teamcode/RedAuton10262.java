// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Auton10262.State.LOWER_ARM;
import static org.firstinspires.ftc.teamcode.Auton10262.State.RAISE_ARM;
import static org.firstinspires.ftc.teamcode.Auton10262.State.RAMP_UP;

/**
 * AutonDrive Mode
 * <p>
 */
@Autonomous(name="Red Auton 10262", group="Pioneer 10262")
public class RedAuton10262 extends Auton10262 {

    /**
     * Constructor
     */
    public RedAuton10262() {
        // most if not all of your setup code
        // belongs in init, not here (see below)
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    protected State handleState(State state, double time_in_state) {
        switch (state) {
            case BEGIN:
                reset_heading();
                state = LOWER_ARM;
                break;

            case FOUND_RED_JEWEL:
                if (enterState) {
                    kicker_ramp.reset(jewel_kicker.getPosition(), Calibration10262.JEWEL_KICK_BACK, (long) (Calibration10262.GEM_DRIVE_DURATION * 1000));
                }
                if (time_in_state < Calibration10262.GEM_DRIVE_DURATION) {
                    kicker_ramp.loop();
                } else {
                    state = RAISE_ARM;
                }
                break;

            case FOUND_BLUE_JEWEL:
                if (enterState) {
                    kicker_ramp.reset(jewel_kicker.getPosition(), Calibration10262.JEWEL_KICK_FORWARD, (long) (Calibration10262.GEM_DRIVE_DURATION * 1000));
                }
                if (time_in_state < Calibration10262.GEM_DRIVE_DURATION) {
                    kicker_ramp.loop();
                } else {
                    state = RAISE_ARM;
                }
                break;

            case RAISE_ARM:
                state = super.handleState(state, time_in_state);
                drive_speed = -drive_speed;
                break;

            default:
                state = super.handleState(state, time_in_state);
        }

        return state;
    }

//    private void imu_loop() {
//        Orientation pos = imu.getAngularOrientation();
//        telemetry.addData("IMU: ", "h:" + pos.firstAngle + " y: " + pos.secondAngle + " p:" + pos.thirdAngle);
//    }

}
