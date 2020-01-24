# ロボットシステム学2019課題2
・Raspberry Piに接続したカメラから撮影したボールの色に応じて,Arduinoに接続したLEDを点灯させるプログラムを作成  
・カメラに近い方のボールの色を認識  
・手順  
  1. Raspberry PiとPCをSSHしてroscoreを起動  
  2. Raspberry piのマスタ権限:ROS_MASTER_URIをPC側にexport  
  3. Raspberry Pi側でカメラ用のlaunchファイル:raspi_usb.launchを実行  
  4. PC側でカメラの色情報をArduino側にpublishするlaunchファイル:usb_cam_raspi.launchを実行  
  5. カメラに使用権限を与える "sudo chmod 777 /dev/ttyUSB0"を実行  
  6. ArduinoのSubscribeするプログラムを実行"rosrun rosserial_python serial_node.py_port:=/dev/ttyUSB0"   
・結果    
  1. 青ボール:LED1つ点灯(4秒点灯)  
  2. 黄ボール:LED2つ点灯(4秒点灯)  
  3. 赤ボール:LED3つ点灯(4秒点灯)  
  
# License
・This repository is lisensed under the BSD-3-Clause License, see LICENSE.

# Demo video
https://www.youtube.com/watch?v=bDiWisYprZM
