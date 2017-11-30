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
@Autonomous(name="Blue Auton 10262", group="Pioneer 10262")
public class BlueAuton10262 extends Auton10262 {
    /**
     * Constructor
     */
    public BlueAuton10262() {
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
                state = LOWER_ARM;
                break;

            case FOUND_BLUE_JEWEL:
                if (time_in_state < Calibration10262.GEM_DRIVE_DURATION) {
                    jewel_kicker.setPosition(Calibration10262.JEWEL_KICK_BACK);
                } else {
                    state = RAISE_ARM;
                }
                break;

            case FOUND_RED_JEWEL:
                if (time_in_state < Calibration10262.GEM_DRIVE_DURATION) {
                    jewel_kicker.setPosition(Calibration10262.JEWEL_KICK_FORWARD);
                } else {
                    state = RAISE_ARM;
                }
                break;

            case RAISE_ARM:
                super.handleState(state, time_in_state);
                jewel_kicker.setPosition(Calibration10262.JEWEL_KICK_CENTER);
                state = RAMP_UP;
                break;

            case RAMP_UP:
                if (time_in_state > Calibration10262.AUTON_DRIVE_RAMP_TIME) {
                    state = State.RAMP_DOWN;
                } else {
                    double percent = time_in_state / Calibration10262.AUTON_DRIVE_RAMP_TIME;
                    double power = Calibration10262.AUTON_DRIVE_SPEED * percent;
                    set_drive_power(power, power);
                }
                break;

            case RAMP_DOWN:
                if (time_in_state > Calibration10262.AUTON_DRIVE_RAMP_TIME) {
                    state = State.STOP;
                } else {
                    double percent = 1.0 - (time_in_state / Calibration10262.AUTON_DRIVE_RAMP_TIME);
                    double power = Calibration10262.AUTON_DRIVE_SPEED * percent;
                    set_drive_power(power, power);
                }
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
