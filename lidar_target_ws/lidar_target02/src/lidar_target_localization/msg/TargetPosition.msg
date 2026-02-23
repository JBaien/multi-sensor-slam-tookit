std_msgs/Header header
float64 x          # 小车相对标靶 X (米), 标靶为原点
float64 y          # 小车相对标靶 Y (米), 标靶为原点
float64 distance   # 小车到标靶直线距离
float64 target_x   # 标靶在激光雷达坐标系 X
float64 target_y   # 标靶在激光雷达坐标系 Y
int32   point_count  # 标靶点云点数
bool    is_valid   # 本帧检测是否有效
