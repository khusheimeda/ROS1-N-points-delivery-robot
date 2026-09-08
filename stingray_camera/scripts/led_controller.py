#!/usr/bin/env python3
"""
LED Controller for Triton robot.

The robot's firmware listens on /cmd_color for an Int32 packed as
24-bit RGB: 0xRRGGBB (e.g. 0xFF0000 = red, 0x00FF00 = green).

This node accepts simple symbolic codes on /led_command and translates
them into the right 24-bit RGB values on /cmd_color. Also runs the
rainbow animation (which has to be a continuous loop, not a one-shot).

Symbolic codes accepted on /led_command (Int32):
  0 = OFF
  1 = RED       (obstacle)
  2 = GREEN     (navigating)
  3 = YELLOW    (delivered, brief)
  4 = BLUE      (idle)
  5 = RAINBOW   (cycle through all colors)
"""
import rospy
from std_msgs.msg import Int32


# Symbolic codes (input on /led_command)
CMD_OFF      = 0
CMD_RED      = 1
CMD_GREEN    = 2
CMD_YELLOW   = 3
CMD_BLUE     = 4
CMD_RAINBOW  = 5
CMD_SIREN    = 6   # 🚨 police siren (rapid red ↔ blue)

# Hex color values (output on /cmd_color, packed as 0xRRGGBB)
HEX_COLORS = {
    CMD_OFF:    0x000000,    # black / off
    CMD_RED:    0xFF00FF,    # 🟣 magenta — used when robot is stopped
    CMD_GREEN:  0x00FF00,    # green
    CMD_YELLOW: 0xFFFF00,    # yellow
    CMD_BLUE:   0x0000FF,    # blue
}

# Siren config — alternates between these two colors
SIREN_COLOR_A = 0xFF0000   # red
SIREN_COLOR_B = 0x0000FF   # blue
SIREN_PERIOD  = 0.15        # seconds per color (smaller = faster flash)


def rgb_to_hex(r, g, b):
    """Pack three 0..1 floats into a 0xRRGGBB int32."""
    R = max(0, min(255, int(r * 255)))
    G = max(0, min(255, int(g * 255)))
    B = max(0, min(255, int(b * 255)))
    return (R << 16) | (G << 8) | B


class LEDController:
    def __init__(self):
        rospy.init_node('led_controller')

        self.update_hz  = rospy.get_param('~update_hz',  30.0)   # render rate
        self.rainbow_hz = rospy.get_param('~rainbow_hz', 0.5)    # cycles/sec

        self.current_code   = -1
        self.rainbow_active = False
        self.rainbow_phase  = 0.0   # 0..1
        self.siren_active   = False
        self.siren_state    = 0     # 0 = color A, 1 = color B
        self.siren_last_t   = 0.0   # last toggle time
        self.last_hex       = None  # avoid spamming the firmware

        rospy.Subscriber('/led_command', Int32, self._cmd_cb)
        self.color_pub = rospy.Publisher('/cmd_color', Int32,
                                         queue_size=1, latch=True)

        rospy.loginfo("LEDController: ready")
        rospy.loginfo(f"  Subscribing to:  /led_command  (symbolic codes)")
        rospy.loginfo(f"  Publishing to:   /cmd_color    (24-bit RGB hex)")
        rospy.loginfo(f"  Update rate:     {self.update_hz} Hz")
        rospy.loginfo(f"  Rainbow speed:   {self.rainbow_hz} cycles/sec")

        # Start with LED off
        self._publish_hex(0x000000)

        # Main loop — rainbow & siren need continuous updates
        rate = rospy.Rate(self.update_hz)
        while not rospy.is_shutdown():
            if self.rainbow_active:
                self._update_rainbow()
            elif self.siren_active:
                self._update_siren()
            rate.sleep()

    # ------------------------------------------------------------------
    def _cmd_cb(self, msg):
        code = msg.data
        if code == self.current_code:
            return
        self.current_code = code

        if code == CMD_RAINBOW:
            rospy.loginfo("🌈 Rainbow mode activated!")
            self.rainbow_active = True
            self.siren_active   = False
            self.rainbow_phase  = 0.0
            return

        if code == CMD_SIREN:
            rospy.loginfo("🚨 Siren mode activated!")
            self.siren_active   = True
            self.rainbow_active = False
            self.siren_state    = 0
            self.siren_last_t   = rospy.get_time()
            self._publish_hex(SIREN_COLOR_A)   # immediate first flash
            return

        # Static color
        self.rainbow_active = False
        self.siren_active   = False
        if code in HEX_COLORS:
            hex_color = HEX_COLORS[code]
            color_names = {
                CMD_OFF: 'OFF', CMD_RED: 'RED', CMD_GREEN: 'GREEN',
                CMD_YELLOW: 'YELLOW', CMD_BLUE: 'BLUE'
            }
            rospy.loginfo(f"LED → {color_names[code]} (0x{hex_color:06X})")
            self._publish_hex(hex_color)
        else:
            rospy.logwarn(f"LEDController: unknown command code {code}")

    # ------------------------------------------------------------------
    def _update_rainbow(self):
        """Smoothly cycle hue 0..1 to drive RGB output."""
        self.rainbow_phase += self.rainbow_hz / self.update_hz
        if self.rainbow_phase >= 1.0:
            self.rainbow_phase -= 1.0

        # HSV(hue, 1, 1) → RGB
        h = self.rainbow_phase * 6.0
        i = int(h)
        f = h - i

        if   i == 0: r, g, b = 1.0,     f,       0.0       # red → yellow
        elif i == 1: r, g, b = 1.0 - f, 1.0,     0.0       # yellow → green
        elif i == 2: r, g, b = 0.0,     1.0,     f         # green → cyan
        elif i == 3: r, g, b = 0.0,     1.0 - f, 1.0       # cyan → blue
        elif i == 4: r, g, b = f,       0.0,     1.0       # blue → magenta
        else:        r, g, b = 1.0,     0.0,     1.0 - f   # magenta → red

        self._publish_hex(rgb_to_hex(r, g, b))

    # ------------------------------------------------------------------
    def _update_siren(self):
        """🚨 Police siren — alternate red/blue every SIREN_PERIOD seconds."""
        now = rospy.get_time()
        if now - self.siren_last_t < SIREN_PERIOD:
            return
        self.siren_last_t = now
        self.siren_state  = 1 - self.siren_state
        color = SIREN_COLOR_A if self.siren_state == 0 else SIREN_COLOR_B
        self._publish_hex(color)

    # ------------------------------------------------------------------
    def _publish_hex(self, hex_color):
        """Publish a 24-bit RGB int to /cmd_color (skip if unchanged)."""
        if hex_color == self.last_hex:
            return
        self.last_hex = hex_color
        self.color_pub.publish(Int32(data=hex_color))


if __name__ == '__main__':
    try:
        LEDController()
    except rospy.ROSInterruptException:
        pass
