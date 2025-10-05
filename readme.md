This project runs on the Waveshare ESP32-S3-Knob-Touch-LCD-1.8[1] 

There are 3 components:
    BlueKnob-ESP32 - This runs on the ESP32 chip. It puts the processor into a permanent deep sleep at boot. This is to maximize battery life, as it is not used.

    BlueKnob-ESP32S3 - This runs on the ESP32S3 chip. It connects to a device (Android, Windows, other) over Bluetooth (BLE) and sends various keyboard and mouse commands, with the primary focus on media playback (volume, pause, next track, etc).

    BlueKnob-UI - This is a Squareline Studio[2] project where the UI is designed for the ESP32S3. Several "fun" backgrounds based on retro media formats (cassette tapes, vinyl, blank CDs) are included and are loaded at random. The entire project is rotated 180 degrees, to keep the USB C port on the rear of the knob for easier use while charging, and to avoid any costly software rotation.

Configure:
    You can edit the UI in Squareline Studio to your liking. You can also change or replace background images.
    Many settings that are not available to change in the UI on the knob are stored in the settings.h file.

Install:
    Build and flash both ESP projects. Instructions for configuring the ESP-IDF build environment and flashing both chips (by reversing the USB C connector) are available at the Waveshare wiki article[3] for the knob.

    Squareline Studio unfortunately stores paths in a non-relative format so you will need to set the "Project Export Root" and "UI Files Export Path" manually. Root should be set to ".../BlueKnob-ESP32S3/main" and UI Files to ".../BlueKnob-ESP32S3/main/ui"

Usage:
    Turn on the knob and open the bluetooth settings on your device (Android, Windows, other).
    Pair with "BlueKnob".
    The bluetooth indicator on the knob screen will be blue when connected, and gray when not.

    Turning the knob to the right increases the volume level.
    Turning the knob to the left decreases the volume level.
    Tapping the "Play" button will play/pause.
    Tapping the "Prev" or "Next" buttons will load the next or previous track.
    Long-pressing "Play" sends the android "screen wake/sleep" command.
    Long pressing "Prev" and "Next" will fast-forward or rewind the current track.
    Pressing the bottom of the screen (below the "Play" button) while turning the knob also fast-forwards and rewinds.

    Swipe the bottom of the playback controls screen to the left to access settings.
    You can adjust the brightness as well as the theme.
    Changing the theme switches the Play, Prev, and Next buttons between black and white.
    You can adjust the time for the screen going blank and the time for device sleeping.
    The knob continues to work while the screen is blank. Tapping the screen will immedately turn the screen on.
    Tapping the screen or turning the knob while the device is sleeping will wake it, which can take a few seconds.
    The battery level is approximate and may bounce up and down slightly at times. It is accurate enough to give you plenty of warning to recharge when the level is low.
    30 minutes after entering device sleep, the knob will enter hibernation. At this point knob turns and screen taps will no longer wake it. The power switch has to be turned off and back on again to boot. This is to maximize battery life.
    Swipe the bottom of the screen right to return to the playback controls.

    Swipe the bottom of the playback controls screen to the right to access keyboard controls.
    The up/down/left/right buttons send the matching keyboard keys.
    The ✓ and X buttons send the enter key and the key combination for the Android "Back" function.
    Long pressing ✓ will open the Android notifications drawer.
    Long pressing X will send the Android "Home" function. 
    Swipe the bottom of the screen left to return to playback controls.
    
    Swipe the bottom of the keyboard controls screen to the right to access mouse controls.
    In this mode the entire touchscreen acts like a mouse trackpad.
    Move the cursor with your finger. Lift your finger and quickly tap to send a left mouse click.
    To-do: Tap-release-tap-and-drag to send "drag and drop/swipe" style actions.
    Press the bottom of the screen and hold very still for 1.5 seconds to return to the playback controls screen.

Also Included:
    /extras/knob-grip-tpu.stl can be 3D printed to add a soft, grippy ring to the knob. The softer the TPU the better. I printed mine with Siraya Tech's TPU Air at 270c, 0.5 extrusion multiplier/flow rate, 3 perimeters, 0% infill.
    /extras/backgrounds.pdn is included to assist in the creation of background images, located in the "asset_library" folder. Note: The UI project is rotated 180 degrees, so do the same to any background images you make.

[1] https://www.waveshare.com/esp32-s3-knob-touch-lcd-1.8.htm
[2] https://squareline.io/
[3] https://www.waveshare.com/wiki/ESP32-S3-Knob-Touch-LCD-1.8