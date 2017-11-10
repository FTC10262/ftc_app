// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Auton10262.State.*;

/**
 * AutonDrive Mode
 * <p>
 */
@Autonomous(name="Auton", group="Pioneer 10262")
public class Auton10262 extends Base10262 {
//    protected static BNO055IMU imu = null;
    protected double GEM_DRIVE_DURATION = 0.9;

    enum State {
        BEGIN,
        LOWER_ARM,
        WAIT_FOR_COLOR_SENSOR,
        READ_COLOR_SENSOR,
        FOUND_BLUE_JEWEL,
        FOUND_RED_JEWEL,
        KNOCK_BLUE_JEWEL,
        KNOCK_RED_JEWEL,
        RAISE_ARM,
        REALIGN_ROBOT,
        RAMP_UP,
        DRIVE,
        RAMP_DOWN,
        STOP,
        FINISHED
    };

    private State current_state = BEGIN;
    private ElapsedTime time;
    private long counter = 0;

    /**
     * Constructor
     */
    public Auton10262() {
        // most if not all of your setup code
        // belongs in init, not here (see below)
    }

    @Override
    public void init() {
        super.init();
        current_state = BEGIN;
        time = new ElapsedTime();

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu.initialize(parameters);

        jewel_arm.setPosition(JEWEL_ARM_RETRACTED);
        counter = 1;
    }

    protected State handleState(State state, double time_in_state) {

        switch (state) {
            case BEGIN:
                break;

            case LOWER_ARM:
                jewel_arm.setPosition(COLOR_SENSOR_POS_DOWN);
                if (time_in_state > 1.0) {
                    return WAIT_FOR_COLOR_SENSOR;
                }
                break;

            case WAIT_FOR_COLOR_SENSOR:
                if (time_in_state > 1) {
                    return READ_COLOR_SENSOR;
                }
                break;

            case READ_COLOR_SENSOR:
                if (jewel_color.red() > jewel_color.blue()) {
                    return FOUND_RED_JEWEL;
                } else {
                    return FOUND_BLUE_JEWEL;
                }

            case KNOCK_BLUE_JEWEL:
                break;

            case KNOCK_RED_JEWEL:
                break;

            case RAISE_ARM:
                jewel_arm.setPosition(COLOR_SENSOR_POS_UP);
                break;

            case REALIGN_ROBOT:
                break;

            case RAMP_UP:
                break;

            case DRIVE:
                break;

            case RAMP_DOWN:
                break;

            case STOP:
                set_drive_power(0,0);
                return FINISHED;

            case FINISHED:
                break;
        }

        return state;
    }

    @Override
    public void loop() {
        double elapsed = time.seconds();

        telemetry.addData("timer: ", elapsed);
        telemetry.addData("State:", "pre " + current_state + " / " + counter);
        State new_state = handleState(current_state, elapsed);
        telemetry.addData("State:", "post " + current_state + " / " + counter);

        counter += 1;

        if (new_state != current_state) {
            time.reset();
            current_state = new_state;
        }
    }

//    private void imu_loop() {
//        Orientation pos = imu.getAngularOrientation();
//        telemetry.addData("IMU: ", "h:" + pos.firstAngle + " y: " + pos.secondAngle + " p:" + pos.thirdAngle);
//    }

}
