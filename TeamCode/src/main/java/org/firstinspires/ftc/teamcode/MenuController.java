package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.util.List;

/**
 * Created by phurley on 11/10/17.
 */

public class MenuController {
    private List<Field> menu_items;
    private int index = 0;

    public MenuController() {
        menu_items = (new Calibration10262().getStaticFields());
    }

    public Field getCurrentField() {
        return menu_items.get(index);
    }

    public Field nextField() {
        index = (index + 1) % menu_items.size();
        return getCurrentField();
    }

    public Field prevField() {
        index = (index - 1) % menu_items.size();
        return getCurrentField();
    }

    private boolean wasPressed = false;
    public void loop(Telemetry telemetry, Gamepad gamepad) {
        try {
            Field field = getCurrentField();
            String name = field.getName();
            String value = null;
            value = field.get(null).toString();
            telemetry.addData("Calibrate", name + ": " + value);

            if (wasPressed) {
                // de-bounce - do nothing
            } else if (gamepad.dpad_up) {
                nextField();

            } else if (gamepad.dpad_down) {
                prevField();
            } else if (gamepad.dpad_left) {
                prevValue();
            } else if (gamepad.dpad_right) {
                nextValue();
            } else if (gamepad.a) {
                setValue(0);
            } else if (gamepad.b) {
                setValue(1);
            } else if (gamepad.left_bumper && gamepad.right_bumper) {
                (new Calibration10262()).writeToFile();
                telemetry.addData("Calibrate", "saved");
            } else if (Math.abs(gamepad.right_stick_x) > 0.1) {
                setValue(gamepad.right_stick_x);
            }
            wasPressed = gamepad.dpad_down || gamepad.dpad_up || gamepad.dpad_right ||
                         gamepad.dpad_left || (gamepad.right_bumper && gamepad.left_bumper) ||
                         gamepad.a || gamepad.b;

        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    private void prevValue() {
        try {
            Field f = getCurrentField();
            Class<?> klass = getCurrentField().getType();
            String value = f.get(null).toString();

            if (klass.equals(Integer.class)) {
                f.setInt(null, Integer.parseInt(value) - 1);
            } else if (klass.equals(Double.class)) {
                f.setDouble(null, Double.parseDouble(value) - 0.05);
            } else if (klass.equals(Boolean.class)) {
                f.setBoolean(null, !Boolean.parseBoolean(value));
            }
        } catch (IllegalAccessException err) {
            err.printStackTrace();
        }
    }

    private void nextValue() {
        try {
            Field f = getCurrentField();
            Class<?> klass = getCurrentField().getType();
            String value = f.get(null).toString();

            if (klass.equals(Integer.class)) {
                f.setInt(null, Integer.parseInt(value) + 1);
            } else if (klass.equals(Double.class)) {
                f.setDouble(null, Double.parseDouble(value) + 0.05);
            } else if (klass.equals(Boolean.class)) {
                f.setBoolean(null, !Boolean.parseBoolean(value));
            }
        } catch (IllegalAccessException err) {
            err.printStackTrace();
        }
    }

    private void setValue(double v) {
        try {
            Field f = getCurrentField();
            Class<?> klass = getCurrentField().getType();
            String value = f.get(null).toString();

            if (klass.equals(Integer.class)) {
                f.setInt(null, (int) Math.round(v));
            } else if (klass.equals(Double.class)) {
                f.setDouble(null, v);
            } else if (klass.equals(Boolean.class)) {
                f.setBoolean(null, Math.abs(v) > 0.25);
            }
        } catch (IllegalAccessException err) {
            err.printStackTrace();
        }
    }
}
