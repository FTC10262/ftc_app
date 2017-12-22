// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Auton10262.State.ADJUST_TRAY1;
import static org.firstinspires.ftc.teamcode.Auton10262.State.ADJUST_TRAY2;
import static org.firstinspires.ftc.teamcode.Auton10262.State.COLLECT_DRIVE;
import static org.firstinspires.ftc.teamcode.Auton10262.State.FORWARD3;
import static org.firstinspires.ftc.teamcode.Auton10262.State.FORWARD4;
import static org.firstinspires.ftc.teamcode.Auton10262.State.NADER_BACKOFF;
import static org.firstinspires.ftc.teamcode.Auton10262.State.NADER_BACKOFF2;
import static org.firstinspires.ftc.teamcode.Auton10262.State.RAMP_UP;
import static org.firstinspires.ftc.teamcode.Auton10262.State.RELEASE_TRAY;
import static org.firstinspires.ftc.teamcode.Auton10262.State.ROTATE_TO;
import static org.firstinspires.ftc.teamcode.Auton10262.State.STOP;
import static org.firstinspires.ftc.teamcode.Auton10262.State.TURN2;

/**
 * AutonDrive Mode
 * Close is Hard
 * <p>
 */
@Autonomous(name="Red Nader", group="Pioneer 10262")
public class RedNader extends RedAuton10262 {
    /**
     * Constructor
     */
    public RedNader() {
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
                state = TURN2;
                break;

            case TURN2:
                state = ROTATE_TO;
                rotate_to = -90;
                exit_state = COLLECT_DRIVE;
                break;

            case COLLECT_DRIVE:
                left_intake.setPower(-Calibration10262.MAX_INTAKE_POWER);
                right_intake.setPower(Calibration10262.MAX_INTAKE_POWER);
                state = RAMP_UP;
                exit_state = NADER_BACKOFF;
                drive_time = Calibration10262.NADAR_COLLECT_TIME;
                drive_speed = Calibration10262.NADAR_COLLECT_SPEED;
                break;

            case NADER_BACKOFF:
                set_drive_power(-Calibration10262.BACKOFF_SPEED, -Calibration10262.BACKOFF_SPEED);
                if (time_in_state > Calibration10262.BACKOFF_TIME) {
                    state = ADJUST_TRAY1;
                }
                break;

            case ADJUST_TRAY1:
              set_tray_angle(Calibration10262.TRAY_DRIVE_POSITION);
              if (time_in_state > Calibration10262.TRAY_ADJUST_TIME) {
                  left_intake.setPower(0);
                  right_intake.setPower(0);
                  wide_open_tray();
              } else if (time_in_state > Calibration10262.TRAY_ADJUST_TIME2) {
                  close_tray();
                  exit_state = FORWARD3;
              }
                break;


            case FORWARD3:
                state = RAMP_UP;
                exit_state = NADER_BACKOFF2;
                drive_time = Calibration10262.NADAR_REVERSE_TIME;
                drive_speed = Calibration10262.NADAR_REVERSE_SPEED;
                break;

            case NADER_BACKOFF2:
                set_drive_power(Calibration10262.BACKOFF_SPEED, Calibration10262.BACKOFF_SPEED);
                if (time_in_state > Calibration10262.BACKOFF_TIME) {
                    state = ADJUST_TRAY2;
                }
                break;

            case ADJUST_TRAY2:
                set_tray_angle(Calibration10262.TRAY_DEPLOY_POSITION);
                if (time_in_state > Calibration10262.TRAY_ADJUST_TIME) {
                    state = RELEASE_TRAY;
                }
                break;

            case RELEASE_TRAY:
                wide_open_tray();
                if (time_in_state > Calibration10262.TRAY_ADJUST_TIME){
                    state = FORWARD4;
                }
                break;

            case FORWARD4:
                set_drive_power(-Calibration10262.BACKOFF_SPEED, -Calibration10262.BACKOFF_SPEED);
                if (time_in_state > Calibration10262.BACKOFF_TIME) {
                    state = STOP;
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
