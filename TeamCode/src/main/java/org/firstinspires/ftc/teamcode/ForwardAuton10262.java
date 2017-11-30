// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * AutonDrive Mode
 * <p>
 */
@Autonomous(name="Forward Auton 10262", group="Pioneer 10262")
public class ForwardAuton10262 extends Auton10262 {
    /**
     * Constructor
     */
    public ForwardAuton10262() {
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
                state = State.RAMP_UP;

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
                    telemetry.addData("In Ramp down", "stop!!!");
                    state = State.FINISHED;
                } else {
                    double percent = 1.0 - (time_in_state / Calibration10262.AUTON_DRIVE_RAMP_TIME);
                    double power = Calibration10262.AUTON_DRIVE_SPEED * percent;
                    telemetry.addData("In Ramp down", power);
                    set_drive_power(power, power);
                    telemetry.addData("Ramp2", power);
                }
                break;

            default:
                state = super.handleState(state, time_in_state);
                break;
        }

        return state;
    }

//    private void imu_loop() {
//        Orientation pos = imu.getAngularOrientation();
//        telemetry.addData("IMU: ", "h:" + pos.firstAngle + " y: " + pos.secondAngle + " p:" + pos.thirdAngle);
//    }

}
