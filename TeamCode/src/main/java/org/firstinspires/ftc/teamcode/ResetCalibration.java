package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by phurley on 12/5/17.
 */

@TeleOp(name="Reset Calibration", group="Teleop")
public class ResetCalibration extends Base10262 {

    @Override
    public void init() {
        (new Calibration10262()).zapFile();
        blow_up();
    }

    public void blow_up() {
        blow_up();
    }
}
