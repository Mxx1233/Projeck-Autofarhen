# 车道偏差信息
# 从 lane_detection_node 发布给 controller_node

# 标准消息头（包含时间戳和坐标系ID）
std_msgs/Header header

# 横向偏差(米)
# 描述：车辆参考点相对于车道中心的横向位移
# 正值 = 车辆在车道中心右侧
# 负值 = 车辆在车道中心左侧
float32 lateral_error

# 航向偏差(弧度)
# 描述：车辆朝向与车道切线方向的夹角
# 正值 = 车辆朝向指向车道右侧
# 负值 = 车辆朝向指向车道左侧
float32 heading_error

# 车道曲率(1/米)
# 描述：当前车道的曲率
# 0 = 直线
# 正值 = 右转弯
# 负值 = 左转弯
float32 curvature
