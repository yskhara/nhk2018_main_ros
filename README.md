# nhk2018_main_ros

Chiba Robot Studio（千葉大学ロボコンサークル）が，NHK学生ロボコン2018に出場した際の制御プログラムのうち，ROS部分のリポジトリ．

ROSのプログラムは，このほかに，次のようなものがあります：
- [nhk2018_navigation](https://github.com/yskhara/nhk2018_navigation)
  - LRFのデータ処理（[amcl](http://wiki.ros.org/amcl)）に使用する地図データ
  - ダミーのオドメトリノード
- [base_controller](https://github.com/yskhara/base_controller)
  - 極めて貧弱な目標位置追従ノード
  - 足回りの各モータの速度指令値計算ノード
- [can_msgs](https://github.com/yskhara/can_msgs)
  - CANデータをROSのトピックとして扱うためのメッセージ定義．


マイコンのプログラムは，[nhk2018_carrier_node](https://github.com/yskhara/nhk2018_carrier_node)
（CRのマイコン）と[NHK2018_Odometry](https://github.com/yskhara/NHK2018_Odometry)（一次動画撮影まで使用したオドメトリノード）があります．
本戦で使用した足回り（TR・CRで共通）やTRのマイコンのソースコードは，本戦2週間前に発生したSSD故障により紛失しました（マイコンのコードはGitHubに上げてなかった；本戦当日には既にソースコードを紛失していた）．
後輩に残す資産が減ってしまったことを，大変残念に思います．
