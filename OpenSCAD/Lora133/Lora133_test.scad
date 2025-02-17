// Copyright (c) 2024 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

include <board.scad>
include <box.scad>
include <gui.scad>

union() {
  if (show_box) Lora133_box(top);
  if (show_vitamins) {
    in_Lora133_board_frame(board_height=true) Lora133_board();
  }
}
