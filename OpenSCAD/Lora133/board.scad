// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

include <ProjectBox/project_box.scad>
include <ProjectBox/oled91.scad>

board_thickness = 1.6;
pad_space = 2.54;
// GardenL33: board_dims = [26*pad_space, 15.5*pad_space, board_thickness];
board_dims = [19.5*pad_space, 15.5*pad_space, board_thickness];

module LoRa133_to_oled_frame(z_translate=true) {
  u = pad_space;
  z_offset = z_translate ? 13 : 0;
  translate([u*3-0.5, 0.6, z_offset]) children();
}

module Lora133_board() {
  u = pad_space;

  // Board imported from KiCad (VRML) -> meshlab
   translate([board_dims[0]/2, board_dims[1]/2, board_dims[2]/2]) color("white")
    import(file="LoRa133.stl", convexity=3);

  LoRa133_to_oled_frame() oled91();
}
