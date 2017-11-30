// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Auton10262.State.DONE;
import static org.firstinspires.ftc.teamcode.Auton10262.State.STOP;

/**
 * AutonDrive Mode
 * <p>
 */
@Autonomous(name="Red Close Glyph Auton 10262", group="Pioneer 10262")
public class RedCloseGlyphAuton10262 extends RedAuton10262 {

    /**
     * Constructor
     */
    public RedCloseGlyphAuton10262() {
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
            case RAMP_DOWN:
                state = super.handleState(state, time_in_state);
                if (state == STOP) {
                    state = ALIGN_TO_CRYTO_BOX;
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
