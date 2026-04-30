# 视觉阈值调节指南（doll / qr_a4）

> 适用启动命令：
>
> ```bash
> roslaunch turn_on_wheeltec_robot vision_recognition_test.launch start_threshold_tuner:=true tuner_profile:=doll
> roslaunch turn_on_wheeltec_robot vision_recognition_test.launch start_threshold_tuner:=true tuner_profile:=qr_a4
> ```

---

## 1. 总原则（先看）

1. **先保召回，再压误检**：先保证“有目标基本都能出框”，再逐步收紧阈值减少误检。  
2. **一次只调一组参数**：颜色范围 -> 面积 -> 比例/形状，避免互相干扰。  
3. **看连续稳定，不看单帧**：至少观察 5~10 秒连续画面。  
4. **有效阈值判断标准**：
   - `effective_score` 持续高
   - `stable_avg` 稳定（不剧烈抖动）
   - 画面中有目标时持续识别，无目标时少误检

---

## 2. doll 阈值调节（最重要优先级）

`doll` 对应参数：
- 蓝色主目标：`lower_blue / upper_blue`
- 白色辅助区域：`lower_white / upper_white`
- 过滤阈值：`min_blue_area / min_white_area / min_white_ratio / blue_padding_ratio`

### 2.1 优先级（按顺序）

### 第 1 优先：蓝色范围（最重要）
先调：
- `BH_min/BH_max`
- `BS_min/BS_max`
- `BV_min/BV_max`

目标：
- 蓝色目标稳定被分割出来（蓝色 mask 连续完整）
- 背景蓝色干扰不过多

经验：
- 漏检多：降低 `BS_min`、`BV_min`，略放宽 `BH`
- 误检多：提高 `BS_min`、`BV_min`，收窄 `BH`

---

### 第 2 优先：白色范围（辅助置信度）
再调：
- `WH_min/WH_max`
- `WS_min/WS_max`
- `WV_min/WV_max`

目标：
- 在蓝色 ROI 附近能稳定检测到白色区域
- 高光、反光不大面积误判

经验：
- 白色丢失：降低 `WV_min` 或提高 `WS_max`
- 白色噪声多：提高 `WV_min`、降低 `WS_max`

---

### 第 3 优先：面积阈值（稳定性的关键）
调：
- `min_blue_area`
- `min_white_area`

目标：
- 近中远距离都能识别
- 小噪点不触发

经验：
- 远距离漏检：适当降低面积阈值
- 背景点误检：提高面积阈值

---

### 第 4 优先：比例与ROI扩展（收尾）
调：
- `min_white_ratio_x1000`（对应 `min_white_ratio`）
- `blue_pad_x100`（对应 `blue_padding_ratio`）

目标：
- 提高“像目标”的一致性
- 减少偶发蓝块误检

---

### 2.2 doll 验收标准（建议）

- 有目标时：`effective_score` 常态 > `0.65`
- 无目标时：`effective_score` 常态 < `0.30`
- `stable_avg` 平稳，不持续锯齿跳变
- 近/中/远三段距离都做 10 秒观察

---

## 3. qr_a4 阈值调节（最重要优先级）

`qr_a4` 对应参数：
- 白纸LAB范围：`L/A/B min-max`
- 过滤阈值：`min_white_area`
- 黑色占比：`min_black_x1000 / max_black_x1000`
- 形状约束：`aspect_min_x100 / aspect_max_x100`

### 3.1 优先级（按顺序）

### 第 1 优先：LAB白色范围（最重要）
先调：
- `L_min/L_max`
- `A_min/A_max`
- `B_min/B_max`

目标：
- 目标白底能稳定形成候选区域
- 背景白墙/地砖不过多进入 mask

经验：
- 候选太少（漏检）：先放宽 `L`，再略放宽 `A/B`
- 候选太多（误检）：先收紧 `A/B`，再收紧 `L`

---

### 第 2 优先：最小白区面积
调：
- `min_white_area`

目标：
- 保留目标区域，过滤小噪声候选

经验：
- 远处目标丢失：降低该值
- 杂散候选太多：提高该值

---

### 第 3 优先：黑色占比区间
调：
- `min_black_x1000`
- `max_black_x1000`

目标：
- 保留“白底+黑块”结构
- 排除全白/全黑或纹理背景

经验：
- 目标被过滤：降低 `min_black` 或提高 `max_black`
- 背景纹理误检：提高 `min_black`，必要时降低 `max_black`

---

### 第 4 优先：长宽比范围（收尾）
调：
- `aspect_min_x100`
- `aspect_max_x100`

目标：
- 允许透视形变
- 排除极细长或极扁平噪声

经验：
- 斜角拍摄漏检：放宽范围
- 背景矩形误检：收紧范围

---

### 3.2 qr_a4 验收标准（建议）

- 有目标时：`accepted >= 1` 稳定持续
- `effective_score` 常态 > `0.70`
- 无目标时：`accepted` 多数时间为 0
- 近/中/远 + 倾斜角度都验证

---

## 4. 实操流程（两种模式通用）

1. 固定光照与相机曝光（先不要边调边改曝光）。  
2. 先让目标居中，完成参数初调。  
3. 移动目标到边缘、远距离、斜角，检查鲁棒性。  
4. 加入干扰背景（同色物体/白墙/反光）压误检。  
5. 按 `s` 保存参数（保存到节点私有参数 `~saved/*`）。

---

## 5. 你最该盯的反馈项（结论）

- **doll 最关键**：
  1) 蓝色分割是否稳定（第一优先）
  2) `effective_score` 与 `stable_avg` 是否持续稳定

- **qr_a4 最关键**：
  1) LAB 白区是否稳定产出有效候选（第一优先）
  2) `accepted` 是否稳定 >=1，且无目标时保持 0

一句话总结：
- `doll` 先把**蓝色主目标**调准；
- `qr_a4` 先把**白底候选**调准；
- 之后再用面积/比例去抑制误检。
