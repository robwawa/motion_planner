std_msgs/Header header

# The PCT arrays use C-order [layer][row][col].  `row` is the PCT x index
# and `col` is the PCT y index; this intentionally matches planner_wrapper.
float64 resolution
float64 center_x
float64 center_y
uint32 rows
uint32 cols
uint32 layers
float32[] traversability
float32[] ground_elevation
uint8[] elevation_valid
