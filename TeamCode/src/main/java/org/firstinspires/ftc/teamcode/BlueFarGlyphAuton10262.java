// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Auton10262.State.COUNTER_ROTATE_TO;
import static org.firstinspires.ftc.teamcode.Auton10262.State.FORWARD0;
import static org.firstinspires.ftc.teamcode.Auton10262.State.FORWARD1;
import static org.firstinspires.ftc.teamcode.Auton10262.State.FORWARD2;
import static org.firstinspires.ftc.teamcode.Auton10262.State.PUSH_GLYPH;
import static org.firstinspires.ftc.teamcode.Auton10262.State.RAMP_UP;
import static org.firstinspires.ftc.teamcode.Auton10262.State.ROTATE_TO;
import static org.firstinspires.ftc.teamcode.Auton10262.State.STOP;
import static org.firstinspires.ftc.teamcode.Auton10262.State.TURN1;
import static org.firstinspires.ftc.teamcode.Auton10262.State.TURN2;

/**
 * AutonDrive Mode
 * Far is easy
 * <p>
 */
@Autonomous(name="Blue Far Glyph Auton 10262", group="Pioneer 10262")
public class BlueFarGlyphAuton10262 extends BlueAuton10262 {

    /**
     * Constructor
     */
    public BlueFarGlyphAuton10262() {
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
            case RAISE_ARM:
                state = super.handleState(state, time_in_state);
                state = FORWARD0;
                break;

            case FORWARD0:
                exit_state = TURN1;
                state = RAMP_UP;
                drive_time = Calibration10262.HARD_FORWARD0_TIME;
                drive_speed = Calibration10262.HARD_FORWARD0_SPEED;
                break;

            case TURN1:
                state = ROTATE_TO;
                rotate_to = 90;
                exit_state = FORWARD1;
                break;

            case FORWARD1:
                exit_state = TURN2;
                state = RAMP_UP;
                drive_time = Calibration10262.HARD_FORWARD1_TIME;
                drive_speed = Calibration10262.HARD_FORWARD1_SPEED;
                break;

            case TURN2:
                exit_state = FORWARD2;
                state = COUNTER_ROTATE_TO;
                rotate_to = 0;
                break;

            case FORWARD2:
                exit_state = PUSH_GLYPH;
                state = RAMP_UP;
                drive_time = Calibration10262.HARD_FORWARD2_TIME;
                drive_speed = Calibration10262.HARD_FORWARD2_SPEED;
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
