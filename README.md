# third_robot_monitor
## How to launch
### service server
- `map_server`で`/image`と`/resolution`パラメータを登録した状態にして下さい。
- `/hogehoge/image`の場合は引数`ns`で指定して下さい。
- 地図ファイルを見に行くフォルダは`path_to_ws/TC2015/third_robot/third_robot_2dnav/map/*`です。
```
roslaunch third_robot_monitor third_robot_monitor_server.launch ns:=hogehoge
```

### service client
```
rosrun third_robot_monitor third_robot_monitor_client
```

## parameters
- `/config/third_robot_monitor.yaml`
- ratio: 0.2
  - 元画像ファイルを表示する際の初期縮小率。

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
