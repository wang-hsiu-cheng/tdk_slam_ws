## 實驗流程
1. 模擬環境建圖與定位，比較不同套件：slam_toolbox, catographer
    |項目|slam_toolbox| catographer|
    |--|--|--|
    |建圖情況|牆壁邊緣有較明顯鋸齒|同個地區需要旋轉掃比較多遍|
    |定位情形|1cm~3cm|10mm ~ 0.8mm|
    |打滑重校正|無法校正|需要來回移動一段時間才能校正|

2. 現實環境雙 LiDAR 建圖
    1. 純 LiDAR 掃描，調整 LiDAR fusion 角度與設定：目前沒問題，不須特別調整
    2. 比較 slam_toolbox & catographer:
        |項目|slam_toolbox|catographer|
        |--|--|--|
        |建圖難易度|||
        |建圖精準度測量|||

3. 現實環境雙 LiDAR 定位
    1. 加入 odometry 回授，比較 slam_toolbox & catographer:
        |項目|slam_toolbox|catographer|
        |--|--|--|
        |定位靜態精度|||
        |定位靜態準度|||
        |瞬間移動後的回復速度|||
        |遮擋容忍程度|||
        |更新速度與延遲|||
    2. 使用 catographer 比較有無 IMU 回授的定位效果
        |項目|odometry |odometry+IMU|
        |---|---|---|
        |整合情況|已完成||
        |定位靜態精度|||
        |定位靜態準度|||
        |瞬間移動後的回復速度|||
        |遮擋容忍程度|||
        |更新速度與延遲|||

4. 有空再進行的額外實驗：使用雙 LiDAR + IMU 進行 SLAM 定位 (無 odometry、及時建圖定位)

## 整合
1. map topic: 
    - localization workspace 發送
    - navigation workspace 接收使用
2. 機器人位置初始化(main program):
    1. SEND_INIT_REQUEST state: 主程式發送抽象初始化訊號(x, y, yaw)
    2. WAITING_FOR_MANAGER state: 等待初始化狀態
        - 成功: state -> SYSTEM_READY
        - 失敗: state -> SEND_INIT_REQUEST
        - 失敗超過 5 次: state -> ALARM_FAILURE
    3. SYSTEM_READY state: 開始主程式後續工作
    4. ALARM_FAILURE state: 初始化失敗，表示狀況有點糟糕
3. 機器人位置初始化(localization_manager):
    1. 非 SUCCESS state: 接收到抽象初始化訊號: 
        - localization_manager 重置底層 SLAM：state -> TRIGGER_SLAM
        - slam_toolbox: /initial_pose topic: state -> VERIFYING
        - cartographer: /start_trajectory service: state -> VERIFYING
    2. VERIFYING state: localization_manager 檢查 /robot_pose 抵達初始化目標
        - 重置成功：呼叫 service 清空 Nav2 全域與局部 Costmap、localization_manager 回傳初始化成功給主程式：state ->SUCCESS
        - 重置失敗：回傳初始化失敗給主程式，等待主程式發送抽象初始化訊號：state -> IDLE
    3. SUCCESS state: 在成功狀態維持 4sec 後繼續接收主程式：state -> IDLE
4. nav2 & speed controller

## 待測試
- 腳本
    - entrypoint.sh 安裝腳本
    - YD LiDAR driver 安裝與編譯(給tdk第一組測試)
- nav2 導航 static layer(給導航負責人整合測試)
    - map server /map topic
    - slam_toolbox /map topic
- 定位初始化節點
    - 將初始化流程整合進主程式