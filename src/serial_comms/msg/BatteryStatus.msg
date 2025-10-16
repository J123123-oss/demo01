float32 total_voltage       # 总电压(V)
float32 current             # 电流(A)，正值为充电，负值为放电
float32 remaining_capacity  # 剩余容量(Ah)
uint8 mos_state
uint8 battery_series
uint8 ntc_count
float32 nominal_capacity    # 标称容量(Ah)
uint16 cycle_count          # 循环次数
uint16 production_year      # 生产年份
uint8 production_month      # 生产月份
uint8 production_day        # 生产日期
uint16 balance_low          # 低16位均衡状态
uint16 balance_high         # 高16位均衡状态
uint16 protection_status    # 保护状态
string software_version     # 软件版本
float32 batttery_remaining     # 电量百分比(0-100)
float32[] temperatures      # 温度传感器数据(°C)
