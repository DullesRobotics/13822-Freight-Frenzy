package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;

import java.nio.ByteBuffer;

import static com.qualcomm.robotcore.robocol.RobocolParsable.HEADER_LENGTH;

public class Controller {

    private Gamepad g;
    private boolean autoMode = false;

    public Controller(@NotNull Gamepad g) {
        this.g = g;
    }

    private boolean a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button, start, back, guide;
    private float left_trigger, right_trigger, left_stick_x, left_stick_y, right_stick_x, right_stick_y;

    public boolean buttonA() {
        return !autoMode ? g.a : a;
    }

    public boolean buttonB() {
        return !autoMode ? g.b : b;
    }

    public boolean buttonX() {
        return !autoMode ? g.x : x;
    }

    public boolean buttonY() {
        return !autoMode ? g.y : y;
    }

    /* Set automated values to false because these controls are used in the robot recorder.
     * We don't want to mimic recorder-specific controller inputs. That would screw things up. */
    public boolean buttonUp() {
        return !autoMode ? g.dpad_up : false;
    }

    public boolean buttonDown() {
        return !autoMode ? g.dpad_down : false;
    }

    public boolean buttonRight() {
        return !autoMode ? g.dpad_right : false;
    }

    public boolean buttonLeft() {
        return !autoMode ? g.dpad_left : false;
    }

    public boolean leftBumper() {
        return !autoMode ? g.left_bumper : left_bumper;
    }

    public boolean rightBumper() {
        return !autoMode ? g.right_bumper : right_bumper;
    }

    public float leftTrigger() {
        float preReading = autoMode ? left_trigger : g.left_trigger;
        float reading = (float) (Math.pow(g.left_trigger, 2));
        if (preReading < 0)
            reading *= -1;
        return reading;
    }

    public float rightTrigger() {
        float preReading = autoMode ? right_trigger : g.right_trigger;
        float reading = (float) (Math.pow(g.right_trigger, 2));
        if (preReading < 0)
            reading *= -1;
        return reading;
    }

    public float leftX() {
        return autoMode ? left_stick_x : g.left_stick_x;
    }

    public float rightX() {
        return autoMode ? right_stick_x : g.right_stick_x;
    }

    public float leftY() {
        return autoMode ? left_stick_y : g.left_stick_y;
    }

    public float rightY() {
        return autoMode ? right_stick_y : g.right_stick_y;
    }

    public boolean leftStickButton() {
        return !autoMode ? g.left_stick_button : left_stick_button;
    }

    public boolean rightStickButton() {
        return !autoMode ? g.right_stick_button : right_stick_button;
    }

    public boolean buttonStart() {
        return !autoMode ? g.start : start;
    }

    public boolean buttonBack() {
        return !autoMode ? g.back : back;
    }

    public boolean buttonMode() {
        return !autoMode ? g.guide : guide;
    }

    /**
     * IF the robot controller is locked
     *
     * @return A boolean that is true if the accessible controller is locked
     */
    public boolean isAutoMode() {
        return autoMode;
    }

    /**
     * Set if the controller should be locked
     *
     * @param autoMode What the controller lock state should be
     */
    public void setAutoMode(boolean autoMode) {
        this.autoMode = autoMode;
    }

    /**
     * Use a controller like it's unlocked
     *
     * @return The standard controller
     */
    public Controller asStandard() {
        return new Controller(g);
    }

    /**
     * Sets the controller values while in auto-mode (not manually controlled)
     *
     * @param byteArray The bytes of a corresponding GamePad object. Get it through {@link com.qualcomm.robotcore.hardware.Gamepad#toByteArray}
     */
    @SuppressWarnings("unused")
    public void setAutoModeValues(byte[] byteArray) throws RobotCoreException {
        final short BUFFER_SIZE = 43 + HEADER_LENGTH;
        if (byteArray.length < BUFFER_SIZE)
            throw new RobotCoreException("Expected buffer of at least " + BUFFER_SIZE + " bytes, received " + byteArray.length);

        int cbHeaderWithoutSeqNum = HEADER_LENGTH - 2;
        ByteBuffer byteBuffer = ByteBuffer.wrap(byteArray, cbHeaderWithoutSeqNum, byteArray.length - cbHeaderWithoutSeqNum);
        //gets rid of first value in byte buffer, used for transmission that we don't need
        byteBuffer.getShort();

        int buttons = 0;
        byte version = byteBuffer.get();

        // extract version 1 values
        if (version >= 1) {
            byteBuffer.getInt(); /* id */
            byteBuffer.getLong(); /* timestamp from boot {@link android.os.SystemClock#uptimeMillis}*/
            left_stick_x = byteBuffer.getFloat();
            left_stick_y = byteBuffer.getFloat();
            right_stick_x = byteBuffer.getFloat();
            right_stick_y = byteBuffer.getFloat();
            left_trigger = byteBuffer.getFloat();
            right_trigger = byteBuffer.getFloat();

            buttons = byteBuffer.getInt();
            boolean touchpad = (buttons & 0x08000) != 0; /* set here to ignore it */
            left_stick_button = (buttons & 0x04000) != 0;
            right_stick_button = (buttons & 0x02000) != 0;
            dpad_up = (buttons & 0x01000) != 0;
            dpad_down = (buttons & 0x00800) != 0;
            dpad_left = (buttons & 0x00400) != 0;
            dpad_right = (buttons & 0x00200) != 0;
            a = (buttons & 0x00100) != 0;
            b = (buttons & 0x00080) != 0;
            x = (buttons & 0x00040) != 0;
            y = (buttons & 0x00020) != 0;
            guide = (buttons & 0x00010) != 0;
            start = (buttons & 0x00008) != 0;
            back = (buttons & 0x00004) != 0;
            left_bumper = (buttons & 0x00002) != 0;
            right_bumper = (buttons & 0x00001) != 0;
        }

        if (version >= 2)
            /*user =*/ byteBuffer.get(); /* user */

        if (version >= 3)
            /*type = Gamepad.Type.values[*/ byteBuffer.get()/*]*/; /* game-pad type */
    }

}
