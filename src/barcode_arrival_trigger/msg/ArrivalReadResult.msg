# 到位（front_marker_control 进入 POS_HOLD）后一次性读码的结果，发布在 ~result。
# 没有总的 success 字段：按 camera/serial 两路各自的 success 判断；
# 也不含 local 位置，到位目标看 front_marker_control 自己的状态话题。
std_msgs/Header header   # 到位时刻（看到 POS_HOLD 跳变的时刻）

# 到位序号，与日志 arrival#N 对应；一旦出现跳变就占号，被忙时忽略、被飞行门控放弃的跳变也占号
#（不发布结果），所以 seq 会跳号，原因见日志 arrival#N ignored / arrival#N 放弃。
uint32 seq

# 到位时刻 debug 话题（/ov13855_apriltag_vision/debug/homogeneous_transform）上最新一条
# 消息的 marker id，须在 marker_timeout_s（默认 0.5 s）以内；过期或还没收到过就用 -1。
int32 marker_id

# camera_called / serial_called 表示本次按 mode 是否调用了这一路，与服务是否在线、
# 是否解码成功无关；未调用的那一路字段保持默认值。服务不在线时对应字段的
# error 记为 unavailable (<原因>)，其余字段为默认值。
bool camera_called
barcode_reader_camera/BarcodeArray camera   # header 为所解图像的 header（同相机节点 ~result），没取到图像时为空
bool serial_called
barcode_reader_serial/BarcodeResult serial  # header 为本地发起读码的时刻（both 时在相机读完之后）

# 读码期间退出了 POS_HOLD：读码不取消，照常发布并在此标记。
bool hold_lost_during_read
