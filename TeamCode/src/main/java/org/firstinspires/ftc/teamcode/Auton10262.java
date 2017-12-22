// Copyright (c) 2017 FTC Team 10262 HippoBotamuses

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.Auton10262.State.*;

/**
 * AutonDrive Mode
 * <p>
 */
@Disabled
public class Auton10262 extends Base10262 {
    protected BNO055IMU imu;
    private double heading_zero = 0;
    protected TimeRampValue kicker_ramp;
    protected boolean enterState = false;

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
        ALIGN_TO_CRYTO_BOX,
        BACKWARD,
        FORWARD,
        FORWARD0,
        FORWARD1,
        FORWARD2,
        BACKWARD1,
        BACKWARD2,

        COLLECT_DRIVE,
        NADER_BACKOFF,
        NADER_BACKOFF2,
        ADJUST_TRAY1,
        FORWARD3,
        ADJUST_TRAY2,
        RELEASE_TRAY,
        FORWARD4,

        TURN1,
        TURN2,
        PUSH_GLYPH,
        ROTATE90,
        COUNTER_ROTATE90,
        ROTATE_TO,
        COUNTER_ROTATE_TO,
        BACKOFF,
        STOP,
        DONE,
        FINISHED
    };

    private Orientation imu_data;

    protected VuforiaLocalizer vuforia;
    protected VuforiaTrackables relicTrackables;
    protected VuforiaTrackable relicTemplate;

    public class VuforiaData {
        public long last_found;
        public RelicRecoveryVuMark vuMark;
        public double tX;
        public double tY;
        public double tZ;
        public double rX;
        public double rY;
        public double rZ;
    };

    private boolean vuforia_initialized = false;
    private boolean imu_initialized = false;
    private boolean looking_for_vumark = false;
    protected VuforiaData vuforia_data = new VuforiaData();
    protected double heading_at_start_of_state = 0;

    private State current_state = BEGIN;
    protected State exit_state = STOP;
    protected double drive_speed = 0;
    protected double drive_time = 0;
    protected double rotate_to = 0;
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

        kicker_ramp= new TimeRampValue(Calibration10262.JEWEL_KICK_CENTER, Calibration10262.JEWEL_KICK_CENTER, 1,
                new Rampable() {
                    @Override
                    public void tick(double value) {
                        jewel_kicker.setPosition(value);
                    }
                });

        // spin up a thread for slow initialization
        if (Calibration10262.INITIALIZE_IMU || Calibration10262.INITIALIZE_VUFORIA) {
            new Thread(new Runnable() {

                @Override
                public void run() {

                    if (Calibration10262.INITIALIZE_VUFORIA) {
                        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

                        parameters.vuforiaLicenseKey = "AXANIyj/////AAAAGbHilBoK6UXQvL1QhufpB9EqnBPl75GN7vP41Y2fMWlDmzLrT4uM/25OLcSHCqijv//NkSz2ERUFGSbvhudYTATEbCbRBB+NOUV5qYaKV7lBk8jy9zzHLeSo5c0NageZDO2kiVyJKppbIoBsm6YErTsHA3VEadrVRll0TOJQw/5p2ibisCmyP/M1OuY49Q+pNqpLF/3gL0wWKDk/0ceM5+84oZoB8GkQE0NIDF2qEmmnLNhafUnCvTcCiblVkeZUiORTRYm2vNBpFRdwKTMBQ8j++NQvB7KIaepGwM2T/budWOCrBEVyKoQ7UcWNM9WXXa57fLo1HfzDiacyekH1dpcHCf3W+cN5BpODJ2WxOMNw";

                        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

                        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
                        relicTemplate = relicTrackables.get(0);
                        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

                        relicTrackables.activate();
                        vuforia_initialized = true;
                    }
                }
            });
        }

        if (Calibration10262.INITIALIZE_IMU) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            // parameters.loggingEnabled = true;
            // parameters.loggingTag = "IMU";
            // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            imu_initialized = true;
        }

        counter = 1;
    }

    protected void vuforia_loop() {
        if (vuforia_initialized) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                looking_for_vumark = false;

                vuforia_data.last_found = System.currentTimeMillis();
                vuforia_data.vuMark = vuMark;

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    vuforia_data.tX = trans.get(0);
                    vuforia_data.tY = trans.get(1);
                    vuforia_data.tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    vuforia_data.rX = rot.firstAngle;
                    vuforia_data.rY = rot.secondAngle;
                    vuforia_data.rZ = rot.thirdAngle;
                }
            }
        }
    }

    protected String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    protected void begin_looking_for_vumark() {
        looking_for_vumark = true;
    }

    @Override
    public void init_loop() {
        super.init_loop();
        jewel_arm.setPosition(Calibration10262.JEWEL_ARM_RETRACTED);

        imu_loop();
        if (looking_for_vumark) {
            vuforia_loop();
        }
    }

    private void imu_loop() {
        if (imu_initialized) {
            imu_data = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
    }

    protected void reset_heading() {
        heading_zero = raw_heading();
    }

    protected double raw_heading() {
        if (imu_initialized && imu_data != null) {
            return imu_data.firstAngle;
        }
        return 0;
    }

    protected double heading() {
        return -AngleUnit.normalizeDegrees(raw_heading() - heading_zero);
    }

    protected State handleState(State state, double time_in_state) {

        switch (state) {
            case BEGIN:
                break;

            case LOWER_ARM:
                jewel_arm.setPosition(Calibration10262.JEWEL_ARM_DEPLOYED);
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

            case FOUND_BLUE_JEWEL:
                break;

            case FOUND_RED_JEWEL:
                break;

            case KNOCK_BLUE_JEWEL:
                break;

            case KNOCK_RED_JEWEL:
                break;

            case RAISE_ARM:
                jewel_arm.setPosition(Calibration10262.JEWEL_ARM_RETRACTED);
                jewel_kicker.setPosition(Calibration10262.JEWEL_KICK_CENTER);
                state = RAMP_UP;
                exit_state = State.DONE;
                drive_time = Calibration10262.AUTON_DRIVE_RAMP_TIME;
                drive_speed = Calibration10262.AUTON_DRIVE_SPEED;
                break;

            case REALIGN_ROBOT:
                break;

            case DRIVE:
                break;

            case ALIGN_TO_CRYTO_BOX:
                break;

            case BACKWARD:
                break;

            case FORWARD:
                if (time_in_state < (drive_time / 2)) {
                    // ramping up
                    double percent = time_in_state / (drive_time / 2);
                    set_drive_power(drive_speed * percent, drive_speed * percent);
                } else if (time_in_state < drive_time) {
                    double percent = (time_in_state - drive_time / 2) / (drive_time / 2);
                    set_drive_power(drive_speed * percent, drive_speed * percent);
                } else {
                    set_drive_power(0,0);
                    state = exit_state;
                }
                break;

            case FORWARD1:
                break;

            case FORWARD2:
                break;

            case BACKWARD1:
                break;

            case BACKWARD2:
                break;

            case TURN1:
                break;

            case TURN2:
                break;

            case PUSH_GLYPH:
                if (time_in_state < Calibration10262.GLYPH_EJECT_TIME) {
                    left_intake.setPower(Calibration10262.GLYPH_EJECT_POWER);
                    right_intake.setPower(-Calibration10262.GLYPH_EJECT_POWER * 2);
                } else {
                    left_intake.setPower(0);
                    right_intake.setPower(0);
                    state = BACKOFF;
                }
                break;

            case ROTATE90: {
                double offset = AngleUnit.normalizeDegrees(AngleUnit.normalizeDegrees(heading_at_start_of_state + 90) - heading());
                telemetry.addData("Rotate90:", offset);
                if (Math.abs((heading_at_start_of_state + 90) - heading()) < Calibration10262.ROTATION_EPSILON) {
                    state = exit_state;
                    set_drive_power(0, 0);
                } else {
                    set_drive_power(Calibration10262.ROTATION_SPEED, -Calibration10262.ROTATION_SPEED);
                }
                break;
            }

            case COUNTER_ROTATE90:
                if (Math.abs((heading_at_start_of_state - 90) - heading()) < Calibration10262.ROTATION_EPSILON) {
                    state = exit_state;
                    set_drive_power(0,0);
                } else {
                    set_drive_power(-Calibration10262.ROTATION_SPEED, Calibration10262.ROTATION_SPEED);
                }
                break;

            case ROTATE_TO: {
                double offset = AngleUnit.normalizeDegrees(rotate_to - heading());
                telemetry.addData("ROTATE_TO:", offset);
                if (Math.abs(offset) < Calibration10262.ROTATION_EPSILON) {
                    state = exit_state;
                    set_drive_power(0, 0);
                } else {
                    set_drive_power(Calibration10262.ROTATION_SPEED, -Calibration10262.ROTATION_SPEED);
                }
                break;
            }

            case COUNTER_ROTATE_TO: {
                double offset = AngleUnit.normalizeDegrees(heading() - rotate_to);
                telemetry.addData("COUNTER ROTATE_TO:", offset);
                if (Math.abs(offset) < Calibration10262.ROTATION_EPSILON) {
                    state = exit_state;
                    set_drive_power(0, 0);
                } else {
                    set_drive_power(-Calibration10262.ROTATION_SPEED, Calibration10262.ROTATION_SPEED);
                }
                break;
            }

            case BACKOFF:
                set_drive_power(-Calibration10262.BACKOFF_SPEED, -Calibration10262.BACKOFF_SPEED);
                if (time_in_state > Calibration10262.BACKOFF_TIME) {
                    state = STOP;
                }
                break;

            case RAMP_UP:
                if (time_in_state > drive_time) {
                    state = State.RAMP_DOWN;
                } else {
                    double percent = time_in_state / drive_time;
                    double power = drive_speed * percent;
                    set_drive_power(power, power);
                }
                break;

            case RAMP_DOWN:
                if (time_in_state > drive_time) {
                    state = exit_state;
                } else {
                    double percent = 1.0 - (time_in_state / drive_time);
                    double power = drive_speed * percent;
                    set_drive_power(power, power);
                }
                break;

            case DONE:
                state = STOP;
                break;

            case STOP:
                set_drive_power(0,0);
                return FINISHED;

            case FINISHED:
                break;
        }

        return state;
    }

    public void resetState() {
        time.reset();
        heading_at_start_of_state = heading();
    }

    public void resetState(State state) {
        time.reset();
        heading_at_start_of_state = heading();
        current_state = state;
    }

    @Override
    public void start() {
        super.start();
        states = "start";
        reset_heading();
    }

    private String states = "";

    @Override
    public void loop() {
        double elapsed = time.seconds();

        imu_loop();
        if (looking_for_vumark) {
            vuforia_loop();
        }

        telemetry.addData("timer: ", elapsed);
        telemetry.addData("State:", "pre " + current_state + " / " + counter);
        State new_state = handleState(current_state, elapsed);
        telemetry.addData("State:", "post " + current_state + " / " + counter);
        telemetry.addData("Heading:", heading());
        telemetry.addData("history:", states);

        counter += 1;

        if (new_state != current_state) {
            resetState();
            enterState = true;
            states += "," + new_state;
            current_state = new_state;
        } else {
            enterState = false;
        }
    }

//    private void imu_loop() {
//        Orientation pos = imu.getAngularOrientation();
//        telemetry.addData("IMU: ", "h:" + pos.firstAngle + " y: " + pos.secondAngle + " p:" + pos.thirdAngle);
//    }

}
