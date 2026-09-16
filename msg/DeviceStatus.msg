std_msgs/Header header

# --- Color stream ---
float64 color_frame_rate_cur
float64 color_frame_rate_avg
float64 color_frame_rate_min
float64 color_frame_rate_max

float64 color_delay_ms_cur
float64 color_delay_ms_avg
float64 color_delay_ms_min
float64 color_delay_ms_max

# --- Left color stream ---
float64 left_color_frame_rate_cur
float64 left_color_frame_rate_avg
float64 left_color_frame_rate_min
float64 left_color_frame_rate_max

float64 left_color_delay_ms_cur
float64 left_color_delay_ms_avg
float64 left_color_delay_ms_min
float64 left_color_delay_ms_max

# --- Right color stream ---
float64 right_color_frame_rate_cur
float64 right_color_frame_rate_avg
float64 right_color_frame_rate_min
float64 right_color_frame_rate_max

float64 right_color_delay_ms_cur
float64 right_color_delay_ms_avg
float64 right_color_delay_ms_min
float64 right_color_delay_ms_max

# --- Depth stream ---
float64 depth_frame_rate_cur
float64 depth_frame_rate_avg
float64 depth_frame_rate_min
float64 depth_frame_rate_max

float64 depth_delay_ms_cur
float64 depth_delay_ms_avg
float64 depth_delay_ms_min
float64 depth_delay_ms_max

# --- Left IR stream ---
float64 left_ir_frame_rate_cur
float64 left_ir_frame_rate_avg
float64 left_ir_frame_rate_min
float64 left_ir_frame_rate_max

float64 left_ir_delay_ms_cur
float64 left_ir_delay_ms_avg
float64 left_ir_delay_ms_min
float64 left_ir_delay_ms_max

# --- Right IR stream ---
float64 right_ir_frame_rate_cur
float64 right_ir_frame_rate_avg
float64 right_ir_frame_rate_min
float64 right_ir_frame_rate_max

float64 right_ir_delay_ms_cur
float64 right_ir_delay_ms_avg
float64 right_ir_delay_ms_min
float64 right_ir_delay_ms_max

# --- Device info ---
bool device_online
string connection_type   # e.g. "USB2.0", "USB3.0", "GigE"

# --- Calibration status ---
bool customer_calibration_ready
bool calibration_from_factory
bool calibration_from_launch_param
