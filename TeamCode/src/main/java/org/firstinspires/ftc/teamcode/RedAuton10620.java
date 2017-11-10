// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Auton10620.State.*;

/**
 * AutonDrive Mode
 * <p>
 */
@Autonomous(name="Red Auton 10620", group="Pioneer 10620")
public class RedAuton10620 extends Auton10620 {
    protected static double RAMP_TIME = 2.0;
    protected static double DRIVE_SPEED = 0.8;

    /**
     * Constructor
     */
    public RedAuton10620() {
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

            case FOUND_RED_JEWEL:
                if (time_in_state < 1.5) {
                    set_drive_power(-0.3, -0.3);
                } else {
                    state = RAISE_ARM;
                }
                break;

            case FOUND_BLUE_JEWEL:
                if (time_in_state < 1.5) {
                    set_drive_power(0.3, 0.3);
                } else {
                    state = RAISE_ARM;
                }
                break;

            case RAISE_ARM:
                super.handleState(state, time_in_state);
                state = RAMP_UP;
                break;

            case RAMP_UP:
                if (time_in_state > RAMP_TIME) {
                    state = State.RAMP_DOWN;
                } else {
                    double percent = time_in_state / RAMP_TIME;
                    double power = DRIVE_SPEED * percent;
                    set_drive_power(-power, -power);
                }
                break;

            case RAMP_DOWN:
                if (time_in_state > RAMP_TIME) {
                    state = State.STOP;
                } else {
                    double percent = 1.0 - (time_in_state / RAMP_TIME);
                    double power = DRIVE_SPEED * percent;
                    set_drive_power(-power, -power);
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
