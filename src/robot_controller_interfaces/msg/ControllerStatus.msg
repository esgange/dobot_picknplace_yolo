std_msgs/Header header
string state
string message
string configuration_id
bool configured
bool tray_configured
bool tray_position_recorded
# Exactly one canonical provider advertises the corresponding pose service.
bool item_detector_ready
bool tray_detector_ready
# Explicit Place permits an empty robot or an item without trusted pickup context.
bool manual_placement_enabled
bool holding_item
bool operation_active
string operation
bool auto_run_active
uint32 auto_run_requested
uint32 auto_run_completed
string phase
string waypoint
uint32 candidate_index
uint32 candidate_total
int16 global_speed_percent
# Last accepted CP ratio (0..100); -1 until known.
int16 global_cp_percent
bool startup_complete
bool feedback_fresh
string[] candidate_ids
string[] candidate_states
bool can_return_item
# Read-only operator eligibility; execution revalidates all safety/source guards.
bool configuration_editable
bool pick_configured
bool motion_ready
string motion_block_reason
bool preview_ready
bool at_tray_detect
bool can_continue
string continue_block_reason
# Observed canonical feedback; valid only while feedback_fresh is true.
bool robot_enabled
bool robot_running
bool robot_queue_active
bool robot_error
bool robot_collision
uint64 digital_input_bits
uint64 digital_outputs
