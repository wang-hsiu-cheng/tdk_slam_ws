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
2. 機器人位置初始化:
    1. 主程式持續發送抽象初始化訊號（目標 B 點）
    2. 接收到抽象初始化訊號: localization_manager 重置底層 SLAM
        - slam_toolbox: /initial_pose topic
        - cartographer: /start_trajectory service
    3. localization_manager 檢查 /robot_pose 抵達 B 點(表示定位程式感測到機器人真的在 B 點)
    4. 重置成功：呼叫 service 清空 Nav2 全域與局部 Costmap、localization_manager 回傳初始化成功給主程式
    5. 主程式停止發送抽象初始化訊號

## 待測試
- 腳本
    - entrypoint.sh 安裝腳本
    - YD LiDAR driver 安裝與編譯
    - save_map 腳本功能
- 地圖
    - map file 檔名拼接語法
    - map_server 遷移後功能
        - map_server autostart 參數功能
        - cartographer 純定位
        - nav2 導航 static layer
    - slam_toolbox /map topic 用於 nav2 導航 static layer
- 定位初始化節點
    - 寫一個pseudo main測試
    - 純定位workspace運行
    - 加入導航節點
    - 確認不會影響現有定位程式執行效能(需要測量&比較)
    - 測量初始化速度(需要測量&比較)
- 建圖調參
    lua 參數調整效果(需要實機測試)