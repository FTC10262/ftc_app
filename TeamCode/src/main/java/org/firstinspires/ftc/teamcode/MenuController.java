package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.util.List;

/**
 * Created by phurley on 11/10/17.
 */

public class MenuController {
    private final ConstantsBase consts;
    private List<Field> menu_items;
    private int index = 0;
    private boolean button_y_pressed;
    private boolean button_x_pressed;

    public MenuController(ConstantsBase consts) {
        menu_items = (consts.getStaticFields());
        this.consts = consts;
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
        if (index < 0) {
            index += menu_items.size();
        }
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
                button_x_pressed = false;
                button_y_pressed = false;
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
                consts.writeToFile();
                telemetry.addData("Calibrate", "saved");
            } else if (gamepad.start) {
                consts.readFromFile();
                telemetry.addData("Calibrate", "reload");
            } else if (Math.abs(gamepad.right_stick_x) > 0.1) {
                setValue(gamepad.right_stick_x);
            } else if (gamepad.x) {
                button_x_pressed = true;
            } else if (gamepad.y) {
                button_y_pressed = true;
            }

            wasPressed = gamepad.dpad_down || gamepad.dpad_up || gamepad.dpad_right ||
                         gamepad.dpad_left || (gamepad.right_bumper && gamepad.left_bumper) ||
                         gamepad.a || gamepad.b || gamepad.start || gamepad.x || gamepad.y;

        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    public boolean buttonX() {
        return button_x_pressed;
    }

    public boolean buttonY() {
        return button_y_pressed;
    }

    private void prevValue() {
        try {
            Field f = getCurrentField();
            Class<?> klass = getCurrentField().getType();
            String value = f.get(null).toString();

            if (klass.equals(Integer.TYPE)) {
                f.setInt(null, Integer.parseInt(value) - 1);
            } else if (klass.equals(Long.TYPE)) {
                f.setLong(null, Long.parseLong(value) - 1);
            } else if (klass.equals(Double.TYPE)) {
                f.setDouble(null, Double.parseDouble(value) - 0.05);
            } else if (klass.equals(Boolean.TYPE)) {
                f.setBoolean(null, !Boolean.parseBoolean(value));
            } else {
                // should probably do something....
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

            if (klass.equals(Integer.TYPE)) {
                f.setInt(null, Integer.parseInt(value) + 1);
            } else if (klass.equals(Long.TYPE)) {
                f.setLong(null, Long.parseLong(value) + 1);
            } else if (klass.equals(Double.TYPE)) {
                f.setDouble(null, Double.parseDouble(value) + 0.05);
            } else if (klass.equals(Boolean.TYPE)) {
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

            if (klass.equals(Integer.TYPE)) {
                f.setInt(null, (int) Math.round(v));
            } else if (klass.equals(Long.TYPE)) {
                f.setLong(null, Math.round(v));
            } else if (klass.equals(Double.TYPE)) {
                f.setDouble(null, v);
            } else if (klass.equals(Boolean.TYPE)) {
                f.setBoolean(null, Math.abs(v) > 0.25);
            } else {
                f.set(null, v);
            }
        } catch (IllegalAccessException err) {
            err.printStackTrace();
        }
    }
}
