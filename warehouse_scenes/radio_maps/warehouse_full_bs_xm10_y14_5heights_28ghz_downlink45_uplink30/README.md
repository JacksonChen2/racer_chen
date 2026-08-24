# 固定 x/y 的五个 BS 高度

BS 平面位置固定为 `(-10, 14) m`，射频高度分别为 5、6、7、8 和 8.5 m。
PHY 为 28 GHz/100 MHz，下行发射功率 45 dBm，上行发射功率 30 dBm。

`warehouse_full_xm10_y14_5_bs_heights_overviews.pdf` 共五页，每个 BS 高度一页，
左侧是五个 UAV 高度层的下行总览，右侧是对应的上行总览。各高度目录保留
NPZ 数值数据和统计 JSON。

`height_ranking.csv` 按五个 UAV 高度层合并后的上行接收功率高于 -120 dBm 的
网格覆盖率降序排列。当前五个候选中，8.5 m 高度最好，覆盖率约为 62.90%。
