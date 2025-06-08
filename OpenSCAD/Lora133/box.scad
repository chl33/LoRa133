// Copyright (c) 2024 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

include <ProjectBox/project_box.scad>
include <ProjectBox/mounts.scad>
include <board.scad>

ones = [1, 1, 1];

wall_thickness = 1;
gap = 0.2;
corner_radius = 2;
mount_offset = pad_space;

space_above_board = 12.5;
space_below_board = 5;
inner_dims = (board_dims
	      + Z*(space_above_board+space_below_board)
	      + 2*gap*ones);
outer_dims = (inner_dims
	      + 2*ones*wall_thickness
	      + [2, 2, 0] * corner_radius);

module Lora33_box_to_board_frame() {
  in_board_frame(outer_dims, board_dims, 0) children();
}


module in_Lora133_board_frame(board_height=false) {
  zoffset = board_height ? space_below_board + wall_thickness - 1: 0;
  in_board_frame(outer_dims, board_dims, zoffset) children();
}

module Lora133_box(top) {
  wall = wall_thickness;
  u = 2.54;

  difference() {
    union() {
      project_box(outer_dims,
		  wall_thickness=wall_thickness,
		  gap=gap,
		  corner_radius=corner_radius,
		  top=top);
      if (top) {
	// Bumps on top etc.
	// translate(b1o) rounded_box(b1d, corner_radius);
      } else {
	// Stuff to add on bottom.
	in_Lora133_board_frame() translate([0, 0, epsilon]) {
	  at_corners(board_dims, mount_offset)
	    screw_mount(space_below_board, wall, 2.5/2);
	}
      }	
    }
    // Cut outs.
    if (top) {
      // OLED
      Lora33_box_to_board_frame() LoRa133_to_oled_frame(z_translate=false) {
	translate([0, u*2, outer_dims[2] - wall -1]) {
	  cube([12, 28, wall+2]);
	}
      }
      // Hole for antenna.
      translate([outer_dims[0]-wall-1, 18, -1]) cube([wall+2, 8, 14]);
      // Hole for usb
      translate([23, -1, wall + space_below_board + board_thickness-2]) cube([9.5, wall+2, 3.5]);
      
      // Negative space of bumps.
      // translate(b1o+[wall, wall, -epsilon]) rounded_box(b1d-wall*[2, 2, 1], corner_radius);
      // translate(b2o+[wall, wall, -epsilon]) rounded_box(b2d-wall*[2, 2, 1], corner_radius);

    }
  }
}
