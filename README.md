# third_robot_monitor
## How to launch
### service server
```
roslaunch third_robot_monitor third_robot_monitor_server.launch
roslaunch third_robot_monitor third_robot_monitor_server.launch map_file_name.yaml
```
- 見に行くフォルダは`hogehoge/TC2015/third_robot/third_robot_2dnav/map/*`です。

### service client
```
rosrun third_robot_monitor third_robot_monitor_client
```

## parameters
- ratio: 0.2
  - 元画像ファイルを表示する際の縮小率。

## command
- 地図画像をアクティブにした状態で下記キーを入力するとモードを切り替えられます。
- `c`: current
  - 現在のロボットの位置のみを地図上に表示します。
- `h`: history
  - 履歴を含んだロボットの位置を地図上に表示します。
- `r`: reset history
  - history モードの履歴をクリアします。
- `p`: plus to ratio
  - 地図を拡大します。historyモードの履歴は削除します。
- `m`: minus to ratio
  - 地図を縮小します。historyモードの履歴は削除します。
