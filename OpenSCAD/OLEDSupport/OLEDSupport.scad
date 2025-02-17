// This is a support for a 0.91" OLED screen above a ESP32-WROOM module on a PCB.
// The support slides onto the end of the screen.

z_below = 8.5;      // Space between the OLED screen at the top of the ESP32 module.
oled_width = 12.5;  // Width (screen height) of the OLED screen, including extra space.
board_z = 2.75;     // Space for the height of the OLED screen.
support_width = 6;  // Width of the support (along the width of the OLED screen.
top_wall = 0.2;     // Thickness of the top of the support, above the screen (thin).
wall = 1;           // Thickness of support walls that are not above the top of the OLED screen.

module OLEDSupport() {
  total_height = z_below + board_z + top_wall;
  difference() {
    // The body of the support.
    cube([support_width, oled_width + 2*wall, total_height]);
    // The space below the board.
    translate([-1, 1, wall]) {
      cube([support_width + 2, oled_width, z_below - 2 * wall]);
    }
    // A space for the screen to slide into.
    translate([-1, 1, z_below]) {
      cube([support_width + 2, oled_width, board_z]);
    }
  }
}

OLEDSupport();
