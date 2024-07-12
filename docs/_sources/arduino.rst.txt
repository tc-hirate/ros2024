.. role:: dir

============================================================
Arduinoの使い方
============================================================

Raspberry PiでArduinoを使えるように設定します。

Arduinoを使って、 LEDを点灯させたり、モーターを動かしたりします。 また、ROSと通信をする方法について説明します。

Arduinoのプログラムはスケッチと呼ばれ、C/C++がベースとなっています。 Pythonとは異なるので注意してください。

開発環境をインストールする
============================================================

arduino-cliをインストールします。

.. code-block:: console

    pi@zumo00:~$  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    Installing in /home/pi/bin
    ARCH=ARM64
    OS=Linux
    Using curl as download tool
    Downloading https://downloads.arduino.cc/arduino-cli/arduino-cli_0.33.1_Linux_ARM64.tar.gz
    arduino-cli not found. You might want to add "/home/pi/bin" to your $PATH
    arduino-cli  Version: 0.33.1 Commit: 347bfeb0 Date: 2023-06-30T16:14:12Z installed successfully in /home/pi/bin

|

arduino-cliのパスを通します。

.. code-block:: console

    pi@zumo00:~$ export PATH=$PATH:$HOME/bin

|

arduino-cliのヘルプを表示します。 （パスが通っていることを確認）

.. code-block:: console

    pi@zumo00:~$ arduino-cli help core
    Arduino core operations.

    Usage:
      arduino-cli core [command]

    Examples:
      arduino-cli core update-index

    Available Commands:
      download     Downloads one or more cores and corresponding tool dependencies.
      install      Installs one or more cores and corresponding tool dependencies.
      list         Shows the list of installed platforms.
      search       Search for a core in Boards Manager.
      uninstall    Uninstalls one or more cores and corresponding tool dependencies if no longer used.
      update-index Updates the index of cores.
      upgrade      Upgrades one or all installed platforms to the latest version.

    Flags:
      -h, --help   help for core

    Global Flags:
          --additional-urls strings   Comma-separated list of additional URLs for the Boards Manager.
          --config-file string        The custom config file (if not specified the default will be used).
          --format string             The output format for the logs, can be: text, json, jsonmini, yaml (default "text")
          --log-file string           Path to the file where logs will be written.
          --log-format string         The output format for the logs, can be: text, json
          --log-level string          Messages with this level and above will be logged. Valid levels are: trace, debug, info, warn, error, fatal, panic
          --no-color                  Disable colored output.
      -v, --verbose                   Print the logs on the standard output.

    Use "arduino-cli core [command] --help" for more information about a command.

|

パスを通すためのコマンドを shellのstartup scriptに書いておきます。

.. code-block:: console

    pi@zumo00:~$ echo "export PATH=$PATH:$HOME/bin" >> ~/.bashrc

|

configuration fileを作成します。

.. code-block:: console

    pi@zumo00:~$ arduino-cli config init
    Config file written to: /home/pi/.arduino15/arduino-cli.yaml

|

LEDを点滅させる
============================================================

Zumo Shieldには「user LED」があります。

Arduinoとは次のように接続されています。

.. csv-table::

    "ピン番号", "Zumo Shieldの機能"
    "13", "LED（LOW：消灯、HIGH：点灯）"

|

詳しくは「User's Guide」を確認してください。

この「user LED」を点滅させるスケッチを作ります。

スケッチの名前は「Led」とします。

次のコマンドを実行して、新しいスケッチを作成してください。

.. code-block:: console

    pi@zumo00:~$ arduino-cli sketch new Arduino/Led
    Sketch created in: /home/pi/Arduino/Led

|

ソースファイルを開いてください。

.. code-block:: console

    pi@zumo00:~$ nano Arduino/Led/Led.ino 

|

編集前。

.. code-block:: c

    void setup() {
    }

    void loop() {
    }

|

編集後。

.. code-block:: c

    void setup() {
      pinMode(13, OUTPUT);
    }

    void loop() {
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
      delay(1000);
    }

|

setup()には、ピンをどのように設定するかを書きます。

LEDがデジタルピン13に接続されているので、ピン13を出力に設定します。

.. code-block:: c

    void setup() {
      pinMode(13, OUTPUT);
    }

|

loop()には、Arduinoの動作を書きます。

Arduinoの電源が切られるまで、loop()は何度も繰り返し実行されます。

ピン13をHIGHにするとLEDは点灯し、LOWにすると消灯します。

delay(1000)は、1秒間（1000ms）何もしないという命令です。

.. code-block:: c

    void loop() {
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
      delay(1000);
    }

|

利用できるプラットフォームとライブラリを更新します。

.. code-block:: console

    pi@zumo00:~$ arduino-cli core update-index
    Downloading index: library_index.tar.bz2 1.06 MiB / 2.44 MiB   4Downloading index: library_index.tar.bz2 downloaded             
    Downloading index: package_index.tar.bz2 0 B / 48.25 KiB    0.00Downloading index: package_index.tar.bz2 downloaded             
    Downloading index: package_index.tar.bz2 0 B / 48.25 KiB    0.00Downloading index: package_index.tar.bz2 downloaded  

|

ArduinoとRaspberry Piを接続してください。

接続したら、次のコマンドを実行して、正しく認識されているか確認します。

.. code-block:: console

    pi@zumo00:~$ arduino-cli board list
    Downloading missing tool builtin:ctags@5.8-arduino11...
    builtin:ctags@5.8-arduino11 downloaded                          
    Installing builtin:ctags@5.8-arduino11...
    Skipping tool configuration....
    builtin:ctags@5.8-arduino11 installed
    Downloading missing tool builtin:serial-discovery@1.4.0...
    builtin:serial-discovery@1.4.0 downloaded                       
    Installing builtin:serial-discovery@1.4.0...
    Skipping tool configuration....
    builtin:serial-discovery@1.4.0 installed
    Downloading missing tool builtin:mdns-discovery@1.0.9...
    builtin:mdns-discovery@1.0.9 61.70 KiB / 2.19 MiB    2.75% 00m07builtin:mdns-discovery@1.0.9 189.70 KiB / 2.19 MiB    8.47% 00m0builtin:mdns-discovery@1.0.9 421.70 KiB / 2.19 MiB   18.83% 00m0builtin:mdns-discovery@1.0.9 901.70 KiB / 2.19 MiB   40.26% 00m0builtin:mdns-discovery@1.0.9 downloaded                         
    Installing builtin:mdns-discovery@1.0.9...
    Skipping tool configuration....
    builtin:mdns-discovery@1.0.9 installed
    Downloading missing tool builtin:serial-monitor@0.13.0...
    builtin:serial-monitor@0.13.0 109.69 KiB / 1.78 MiB    6.02% 00mbuiltin:serial-monitor@0.13.0 237.69 KiB / 1.78 MiB   13.05% 00mbuiltin:serial-monitor@0.13.0 525.69 KiB / 1.78 MiB   28.86% 00mbuiltin:serial-monitor@0.13.0 downloaded                        
    Installing builtin:serial-monitor@0.13.0...
    Skipping tool configuration....
    builtin:serial-monitor@0.13.0 installed
    Downloading missing tool builtin:dfu-discovery@0.1.2...
    builtin:dfu-discovery@0.1.2 downloaded                          
    Installing builtin:dfu-discovery@0.1.2...
    Skipping tool configuration....
    builtin:dfu-discovery@0.1.2 installed
    Port         Protocol Type              Board Name  FQBN            Core       
    /dev/ttyACM0 serial   Serial Port (USB) Arduino Uno arduino:avr:uno arduino:avr
    /dev/ttyAMA0 serial   Serial Port       Unknown     

|

arduino:avrのplatform coreをインストールします。

.. code-block:: console

    pi@zumo00:~$ arduino-cli core install arduino:avr
    Downloading packages...
    arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 0 B / 36.28 MiB    0.0arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 61.80 KiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 237.80 KiB / 36.28 MiBarduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 477.80 KiB / 36.28 MiBarduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 999.95 KiB / 36.28 MiBarduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 1.99 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 1.99 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 3.19 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 4.39 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 5.60 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 6.97 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 6.97 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 8.20 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 9.56 MiB / 36.28 MiB  arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 10.90 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 12.08 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 12.08 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 13.42 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 14.74 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 16.06 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 17.35 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 17.35 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 18.73 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 19.59 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 20.70 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 22.04 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 22.04 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 23.22 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 24.50 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 25.48 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 26.96 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 26.96 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 28.31 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 29.52 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 30.69 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 31.74 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 31.74 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 33.06 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 34.31 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 35.67 MiB / 36.28 MiB arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 downloaded            
    arduino:avrdude@6.3.0-arduino17 downloaded                      
    arduino:arduinoOTA@1.3.0 downloaded                             
    arduino:avr@1.8.6 downloaded                                    
    Installing arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7...
    Configuring tool....
    arduino:avr-gcc@7.3.0-atmel3.6.1-arduino7 installed
    Installing arduino:avrdude@6.3.0-arduino17...
    Configuring tool....
    arduino:avrdude@6.3.0-arduino17 installed
    Installing arduino:arduinoOTA@1.3.0...
    Configuring tool....
    arduino:arduinoOTA@1.3.0 installed
    Installing platform arduino:avr@1.8.6...
    Configuring platform....
    Platform arduino:avr@1.8.6 installed

|

正しくインストールされたか確認します。

.. code-block:: console

    pi@zumo00:~$ arduino-cli core list
    ID          Installed Latest Name              
    arduino:avr 1.8.6     1.8.6  Arduino AVR Boards

|

スケッチをコンパイルします。

.. code-block:: console

    pi@zumo00:~$ arduino-cli compile --fqbn arduino:avr:uno Arduino/Led
    Sketch uses 924 bytes (2%) of program storage space. Maximum is 32256 bytes.
    Global variables use 9 bytes (0%) of dynamic memory, leaving 2039 bytes for local variables. Maximum is 2048 bytes.

    Used platform Version Path                                                   
    arduino:avr   1.8.6   /home/pi/.arduino15/packages/arduino/hardware/avr/1.8.6

|

アップロードします。

.. code-block:: console

    pi@zumo00:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino/Led
    avrdude: ser_open(): can't open device "/dev/ttyACM0": Permission denied
    Failed uploading: uploading error: exit status 1

|

/dev/ttyACM0の権限でエラーが出ました。

次のコマンドを実行して、piをdialoutグループに追加します。

.. code-block:: console

    pi@zumo00:~$ sudo usermod -a -G dialout pi
    [sudo] password for pi: 

|

ここで、再起動してください。

.. code-block:: console

    pi@zumo00:~$ sudo shutdown -r now

|

改めて、アップロードします。

.. code-block:: console

    pi@zumo00:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino/Led
    Compiled sketch not found in /tmp/arduino/sketches/6041595C4279387C991DB6762B1AD02E
    pi@zumo00:~$ arduino-cli compile --fqbn arduino:avr:uno Arduino/Led
    Sketch uses 924 bytes (2%) of program storage space. Maximum is 32256 bytes.
    Global variables use 9 bytes (0%) of dynamic memory, leaving 2039 bytes for local variables. Maximum is 2048 bytes.

    Used platform Version Path                                                   
    arduino:avr   1.8.6   /home/pi/.arduino15/packages/arduino/hardware/avr/1.8.6

|

2秒周期でLEDが点滅（1秒点灯、1秒消灯）していることを確認してください。

|

押しボタンスイッチを使ってLEDを点灯させる
============================================================

Zumo Shieldには「user pushbutton」があります。

Arduinoとは次のように接続されています。

.. csv-table::

    "ピン番号", "Zumo Shieldの機能"
    "12", "押しボタンスイッチ（LOW：押されている、HIGH：押されていない）"

|

詳しくは「User's Guide」を確認してください。

この「user pushbutton」が押されているときに「user LED」を点灯し、押されていないときに消灯するスケッチを作ります。

スケッチの名前は「Button」とします。

|

スケッチの作成。

.. code-block:: console

    pi@zumo00:~$ arduino-cli sketch new Arduino/Button
    Sketch created in: /home/pi/Arduino/Button

|

ソースファイルを開く。

.. code-block:: console

    pi@zumo00:~$ nano Arduino/Button/Button.ino 

|

編集。

.. code-block:: c

    int val = 0;

    void setup() {
      pinMode(13, OUTPUT);
      pinMode(12, INPUT_PULLUP);
    }

    void loop() {
      val = digitalRead(12);

      if (val == LOW) {
        digitalWrite(13, HIGH);
      } else {
        digitalWrite(13, LOW);
      }
    }

|

pushbuttonが押されているかどうかを記憶しておくための変数を定義しています。

.. code-block:: c

    int val = 0;

|

pushbuttonがデジタルピン12に接続されているので、ピン12を入力、プルアップ抵抗を有効に設定しています。

.. code-block:: c

    pinMode(12, INPUT_PULLUP);

|

pushbuttonが押されているかどうかを読み込んでいます。

.. code-block:: c

    val = digitalRead(12);

|

コンパイル。

.. code-block:: console

    pi@zumo00:~$ arduino-cli compile --fqbn arduino:avr:uno Arduino/Button/
    Sketch uses 892 bytes (2%) of program storage space. Maximum is 32256 bytes.
    Global variables use 9 bytes (0%) of dynamic memory, leaving 2039 bytes for local variables. Maximum is 2048 bytes.

    Used platform Version Path                                                   
    arduino:avr   1.8.6   /home/pi/.arduino15/packages/arduino/hardware/avr/1.8.6

|

アップロード。

.. code-block:: console

    pi@zumo00:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino/Button/

|

「user pushbutton」を押したときに「user LED」を点灯することを確認してください。

|

モータを動かす
============================================================

Zumo Shieldには2つのモータがあります。 

Arduinoとは次のように接続されています。

.. csv-table::

    "ピン番号", "Zumo Shieldの機能"
    "7", "右モータの方向制御（LOW：前進、HIGH：後進）"
    "8", "左モータの方向制御（LOW：前進、HIGH：後進）"
    "9", "右モータのPWM（0〜100）"
    "10", "左モータのPWM（0〜100）"

|


詳しくは「User's Guide」を確認してください。

「user pushbutton」が押されているときに前進し、押されていないときに停止するスケッチを作ります。

スケッチの名前は「Motor」とします。

|

スケッチの作成。

.. code-block:: console

    pi@zumo00:~$ arduino-cli sketch new Arduino/Motor
    Sketch created in: /home/pi/Arduino/Motor

|

ソースファイルを開く。

.. code-block:: console

    pi@zumo00:~$ nano Arduino/Motor/Motor.ino 

|

編集。

.. code-block:: c

    const int DIRECTION_R = 7;
    const int DIRECTION_L = 8;
    const int PWM_R = 9;
    const int PWM_L = 10;

    const int LED = 13;
    const int BUTTON = 12;

    int val = 0;

    void setup() {
      pinMode(DIRECTION_R, OUTPUT);
      pinMode(DIRECTION_L, OUTPUT);
      pinMode(PWM_R, OUTPUT);
      pinMode(PWM_L, OUTPUT);

      pinMode(LED, OUTPUT);
      pinMode(BUTTON, INPUT_PULLUP);
    }

    void loop() {
      val = digitalRead(BUTTON);

      digitalWrite(DIRECTION_R, LOW);
      digitalWrite(DIRECTION_L, LOW);

      if (val == LOW) {
        analogWrite(PWM_R, 30);
        analogWrite(PWM_L, 30);

        digitalWrite(LED, HIGH);
      } else {
        analogWrite(PWM_R, 0);
        analogWrite(PWM_L, 0);

        digitalWrite(LED, LOW);
      }
    }

|

ピン番号を数字で入力していると間違えることがあるので、それぞれのピンに名前をつけます。

.. code-block:: c

    const int DIRECTION_R = 7;
    const int DIRECTION_L = 8;
    const int PWM_R = 9;
    const int PWM_L = 10;

    const int LED = 13;
    const int BUTTON = 12;

|

ピンの名前を使って、ピンの設定をしています。

.. code-block:: c

    pinMode(DIRECTION_R, OUTPUT);
    pinMode(DIRECTION_L, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(PWM_L, OUTPUT);

    pinMode(LED, OUTPUT);
    pinMode(BUTTON, INPUT_PULLUP);

|

前進するときは各モータの方向制御にLOWを入力します。

.. code-block:: c

    digitalWrite(DIRECTION_R, LOW);
    digitalWrite(DIRECTION_L, LOW);

|

PWM出力するときはanalogWrite(ピン番号, PWM値)を使います。PWM値には0〜100を設定します。

.. code-block:: c

    analogWrite(PWM_R, 30);
    analogWrite(PWM_L, 30);

|

コンパイル。

.. code-block:: console

    pi@zumo00:~$ arduino-cli compile --fqbn arduino:avr:uno Arduino/Motor/
    Sketch uses 1178 bytes (3%) of program storage space. Maximum is 32256 bytes.
    Global variables use 11 bytes (0%) of dynamic memory, leaving 2037 bytes for local variables. Maximum is 2048 bytes.

    Used platform Version Path                                                   
    arduino:avr   1.8.6   /home/pi/.arduino15/packages/arduino/hardware/avr/1.8.6

|

アップロード。

.. code-block:: console

    pi@zumo00:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino/Motor/

|

「user pushbutton」を押したときにモーターが動くことを確認してください。

|

Arduinoと通信する
============================================================

ROS1では、rosserialというツールを使ってtopicを送受信することができました。

ROS2にもros2arduinoというツールがあるのですが、Auduino UNOには対応していないようです。

ここでは、Pythonでシリアル通信をするためのモジュールpyserialを使います。

pyserialはpipを使ってインストールします。pipはPythonのパッケージを管理するためのツールです。

.. code-block:: console

    pi@zumo00:~$ sudo apt install python3-pip
    pi@zumo00:~$ python3 -m pip install pyserial

|

ターミナルから0または1を入力して、0ならばLEDを消灯、1ならばLEDを点灯するプログラムを作ります。

Pythonのプログラムを保存するために、ホームディレクトリにPythonという名前の ディレクトリを作ります。

.. code-block:: console

    pi@zumo00:~$ mkdir Python

|

ここにserial_test.pyという名前でファイルを作ります。

.. code-block:: console

    pi@zumo00:~$ nano Python/serial_test.py

|

編集。

.. code-block:: python

    import serial

    def main():
      print("Open Port")

      ser = serial.Serial()
      ser.port = "/dev/ttyACM0"
      ser.baudrate = 9600
      ser.open()

      while True:
        try:
          cmd = input("type 0 or 1: ")
          if cmd == "1":
            ser.write(b"1")
          else:
            ser.write(b"0")
        except KeyboardInterrupt:
          break

      print("Close Port")
      ser.close()

    if __name__ == '__main__':
      main()

|

試しに実行してみましょう。

.. code-block:: console

    pi@zumo00:~$ python3 Python/serial_test.py
    Open Port
    type 0 or 1: 1
    type 0 or 1: 0
    type 0 or 1: ^CClose Port

|

次にArduinoのプログラムを作ります。

スケッチの名前は「SerialTest」とします。

|

スケッチの作成。

.. code-block:: console

    pi@zumo00:~$ arduino-cli sketch new Arduino/SerialTest
    Sketch created in: /home/pi/Arduino/SerialTest

|

ソースファイルを開く。

.. code-block:: console

    pi@zumo00:~$ nano Arduino/SerialTest/SerialTest.ino 

|

編集。

.. code-block:: c

    void setup() {
        Serial.begin(9600);
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
    }

    void loop() {
        byte var;
        var = Serial.read();
        switch(var) {
            case '0':
                digitalWrite(LED_BUILTIN, LOW);
                break;
            case '1':
                digitalWrite(LED_BUILTIN, HIGH);
                break;
            default:
                break;
        }
    }

|

コンパイル。

.. code-block:: console

    pi@zumo00:~$ arduino-cli compile --fqbn arduino:avr:uno Arduino/SerialTest/
    Sketch uses 1754 bytes (5%) of program storage space. Maximum is 32256 bytes.
    Global variables use 184 bytes (8%) of dynamic memory, leaving 1864 bytes for local variables. Maximum is 2048 bytes.

    Used platform Version Path                                                   
    arduino:avr   1.8.6   /home/pi/.arduino15/packages/arduino/hardware/avr/1.8.6

|

アップロード。

.. code-block:: console

    pi@zumo00:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino/SerialTest/

|

ターミナルから入力した数値に応じてLEDが点灯／消灯するか確認してください。

|

演習2「ジョイスティックを使ってZumoを動かす」
============================================================

（１）LEDの点灯
------------------------------------------------------------

ジョイスティックのAボタンを押すとLEDが点灯するプログラムを作ってください。

Pythonのプログラムは、zm_testパッケージの「serial_led.py」とします。

また、スケッチの名前は「SerialLed」とします。

|

ワークスペースの作成

.. code-block:: console

    pi@zumo00:~$ mkdir -p ~/ros2_ws/src

|

パッケージの作成

.. code-block:: console

    pi@zumo00:~$ cd ~/ros2_ws/src/
    pi@zumo00:~/ros2_ws/src$ ros2 pkg create --build-type ament_python zm_test
    going to create a new package
    package name: zm_test
    destination directory: /home/pi/ros2_ws/src
    package format: 3
    version: 0.0.0
    description: TODO: Package description
    maintainer: ['pi <pi@todo.todo>']
    licenses: ['TODO: License declaration']
    build type: ament_python
    dependencies: []
    creating folder ./zm_test
    creating ./zm_test/package.xml
    creating source folder
    creating folder ./zm_test/zm_test
    creating ./zm_test/setup.py
    creating ./zm_test/setup.cfg
    creating folder ./zm_test/resource
    creating ./zm_test/resource/zm_test
    creating ./zm_test/zm_test/__init__.py
    creating folder ./zm_test/test
    creating ./zm_test/test/test_copyright.py
    creating ./zm_test/test/test_flake8.py
    creating ./zm_test/test/test_pep257.py

    [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
    It is recommended to use one of the ament license identifiers:
    Apache-2.0
    BSL-1.0
    BSD-2.0
    BSD-2-Clause
    BSD-3-Clause
    GPL-3.0-only
    LGPL-3.0-only
    MIT
    MIT-0

|

ワークスペースに移動

.. code-block:: console

    pi@zumo00:~/ros2_ws/src$ cd ~/ros2_ws/

|

serial_led.pyの作成。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ nano src/zm_test/zm_test/serial_led.py

|

編集。

.. code-block:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy

    import serial

    class JoyLed(Node):

        def __init__(self):
            super().__init__('joy_led')
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.subscription = self.create_subscription(
                Joy,
                'joy',
                self.joy_callback,
                10)
            self.subscription

            self.get_logger().info('Open Port')
            self.ser = serial.Serial()
            self.ser.port = "/dev/ttyACM0"
            self.ser.baudrate = 9600
            self.ser.open()

        def joy_callback(self, joy_msg):
            twist = Twist()
            if joy_msg.buttons[0] == 1:
                self.get_logger().info('LED ON')
                self.ser.write(b"1")
            elif joy_msg.buttons[0] == 0:
                self.get_logger().info('LED OFF')
                self.ser.write(b"0")
            else:
                pass
            self.publisher_.publish(twist)

    def main(args=None):
        rclpy.init(args=args)

        joy_led = JoyLed()

        rclpy.spin(joy_led)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        joy_led.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

|

package.xmlを開く。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ nano src/zm_test/package.xml

|

編集。

.. code-block:: none
    :emphasize-lines: 10-13

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematyp>
    <package format="3">
    <name>zm_test</name>
    <version>0.0.0</version>
    <description>TODO: Package description</description>
    <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>
    <license>TODO: License declaration</license>

    <exec_depend>rclpy</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <exec_depend>geometry_msgs</exec_depend>
    <exec_depend>sensor_msgs</exec_depend>

    <test_depend>ament_copyright</test_depend>
    <test_depend>ament_flake8</test_depend>
    <test_depend>ament_pep257</test_depend>
    <test_depend>python3-pytest</test_depend>

    <export>
        <build_type>ament_python</build_type>
    </export>
    </package>

|

setup.pyを開く。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ nano src/zm_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 1, 2, 16, 27

    import os
    from glob import glob

    from setuptools import find_packages, setup

    package_name = 'zm_test'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*_launch.py')),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='ubuntu',
        maintainer_email='ubuntu@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'joy_led = zm_test.serial_led:main',
            ],
        },
    )

|

ビルド。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ sudo apt install python3-colcon-common-extensions
    pi@zumo00:~/ros2_ws$ colcon build --packages-select zm_test
    Starting >>> zm_test 
    --- stderr: zm_test                    
    /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
    warnings.warn(
    ---
    Finished <<< zm_test [9.60s]

    Summary: 1 package finished [11.6s]
    1 package had stderr output: zm_test

|

セットアップファイルの反映。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ source install/local_setup.bash

|

zm_testパッケージのjoy_ledノードの実行

.. code-block:: console

    pi@zumo00:~/ros2_ws$ ros2 run zm_test joy_led

|

joyパッケージのjoy_nodeの実行

.. code-block:: console

    ubuntu@mbc084:~$ ros2 run joy joy_node

|

（２）ジョイスティックの方向キーでzumoを動かす
------------------------------------------------------------

ジョイスティックの方向キーでZumoを操縦するプログラムを作ってください。

Pythonのプログラムは、zm_testパッケージの「serial_motor.py」とします。

また、スケッチの名前は「SerialMotor」とします。

|

スケッチの作成。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ cd
    pi@zumo00:~$ arduino-cli sketch new Arduino/SerialMotor
    Sketch created in: /home/pi/Arduino/SerialMotor

|

ソースファイルを開く。

.. code-block:: console

    pi@zumo00:~$ nano Arduino/SerialMotor/SerialMotor.ino

|

編集。

.. code-block:: c

    const int DIRECTION_R = 7;
    const int DIRECTION_L = 8;
    const int PWM_R = 9;
    const int PWM_L = 10;
            
    const int LED = 13;
    const int BUTTON = 12;
            
    byte val = 0;
            
    void setup() {
        Serial.begin(9600);
        pinMode(LED, OUTPUT);
        digitalWrite(LED, LOW);
    }
            
    void loop() {
        val = Serial.read();
        switch(val) {
            case '0':  // Stop
                digitalWrite(LED, LOW);
                analogWrite(PWM_R, 0);
                analogWrite(PWM_L, 0);
                break;
            case '1':  // Forward
                digitalWrite(LED, HIGH);
                digitalWrite(DIRECTION_R, LOW);
                digitalWrite(DIRECTION_L, LOW);
                analogWrite(PWM_R, 100);
                analogWrite(PWM_L, 100);
                break;
            case '2':  // Backward
                digitalWrite(LED, HIGH);
                digitalWrite(DIRECTION_R, HIGH);
                digitalWrite(DIRECTION_L, HIGH);
                analogWrite(PWM_R, 100);
                analogWrite(PWM_L, 100);
                break;
            case '3':  // Left
                digitalWrite(LED, HIGH);
                digitalWrite(DIRECTION_R, LOW);
                digitalWrite(DIRECTION_L, LOW);
                analogWrite(PWM_R, 100);
                analogWrite(PWM_L, 0);
                break;
            case '4':  // Right
                digitalWrite(LED, HIGH);
                digitalWrite(DIRECTION_R, LOW);
                digitalWrite(DIRECTION_L, LOW);
                analogWrite(PWM_R, 0);
                analogWrite(PWM_L, 100);
                break;
            default:
                break;
        }
    }

|

コンパイル。

.. code-block:: console

    pi@zumo00:~$ arduino-cli compile --fqbn arduino:avr:uno Arduino/SerialMotor/
    Sketch uses 2094 bytes (6%) of program storage space. Maximum is 32256 bytes.
    Global variables use 184 bytes (8%) of dynamic memory, leaving 1864 bytes for local variables. Maximum is 2048 bytes.

    Used platform Version Path                                                   
    arduino:avr   1.8.6   /home/pi/.arduino15/packages/arduino/hardware/avr/1.8.6

|

アップロード。

.. code-block:: console

    pi@zumo00:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino/SerialMotor/

|

serial_motor.pyの作成

.. code-block:: console

    pi@zumo00:~$ cd ros2_ws/
    pi@zumo00:~/ros2_ws$ nano src/zm_test/zm_test/serial_motor.py

|

編集。

.. code-block:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy

    import serial

    class JoyMotor(Node):

        def __init__(self):
            super().__init__('joy_motor')
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.subscription = self.create_subscription(
                Joy,
                'joy',
                self.joy_callback,
                10)
            self.subscription

            self.get_logger().info('Open Port')
            self.ser = serial.Serial()
            self.ser.port = "/dev/ttyACM0"
            self.ser.baudrate = 9600
            self.ser.open()

        def __del__(self):
            self.get_logger().info('Close Port')

        def joy_callback(self, joy_msg):
            twist = Twist()
            if joy_msg.axes[7] == 1:  # 上が押されたら前進
                self.get_logger().info('Forward')
                self.ser.write(b"1")
            elif joy_msg.axes[7] == -1:  # 下が押されたら後進
                self.get_logger().info('Backward')
                self.ser.write(b"2")
            elif joy_msg.axes[6] == 1:  # 左が押されたら左に曲がる
                self.get_logger().info('Left')
                self.ser.write(b"3")
            elif joy_msg.axes[6] == -1:  # 右が押されたら右に曲がる
                self.get_logger().info('Right')
                self.ser.write(b"4")
            else:  # それ以外のときは停止
                self.get_logger().info('Stop')
                self.ser.write(b"0")
            self.publisher_.publish(twist)

    def main(args=None):
        rclpy.init(args=args)

        joy_motor = JoyMotor()

        rclpy.spin(joy_motor)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        self.get_logger().info('Close Port')
        joy_motor.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

|

setup.pyを開く。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ nano src/zm_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 28

    import os
    from glob import glob

    from setuptools import find_packages, setup

    package_name = 'zm_test'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*_launch.py')),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='ubuntu',
        maintainer_email='ubuntu@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'joy_led = zm_test.serial_led:main',
                'joy_motor = zm_test.serial_motor:main',
            ],
        },
    )

|

ビルド。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ colcon build --packages-select zm_test
    Starting >>> zm_test 
    --- stderr: zm_test                    
    /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
    warnings.warn(
    ---
    Finished <<< zm_test [8.76s]

    Summary: 1 package finished [10.3s]
    1 package had stderr output: zm_test

|

セットアップファイルの反映。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ source install/local_setup.bash

|

zm_testパッケージのjoy_motorノードの実行

.. code-block:: console

    pi@zumo00:~/ros2_ws$ ros2 run zm_test joy_motor

|

joyパッケージのjoy_nodeの実行

.. code-block:: console

    ubuntu@mbc084:~$ ros2 run joy joy_node

|

（３）ジョイスティックのアナログスイッチでzumoを動かす
------------------------------------------------------------

ジョイスティックの左アナログスイッチでZumoを操縦するプログラムを作ってください。 

Pythonのプログラムは、zm_testパッケージの「analog_motor.py」とします。

また、スケッチの名前は「AnalogMotor」とします。

|

スケッチの作成。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ cd
    pi@zumo00:~$ arduino-cli sketch new Arduino/AnalogMotor
    Sketch created in: /home/pi/Arduino/AnalogMotor

|

ソースファイルを開く。

.. code-block:: console

    pi@zumo00:~$ nano Arduino/AnalogMotor/AnalogMotor.ino

|

編集。

.. code-block:: c

    T.B.A.

|

コンパイル。

.. code-block:: console

    pi@zumo00:~$ arduino-cli compile --fqbn arduino:avr:uno Arduino/AnalogMotor/AnalogMotor.ino
    Sketch uses 2094 bytes (6%) of program storage space. Maximum is 32256 bytes.
    Global variables use 184 bytes (8%) of dynamic memory, leaving 1864 bytes for local variables. Maximum is 2048 bytes.

    Used platform Version Path                                                   
    arduino:avr   1.8.6   /home/pi/.arduino15/packages/arduino/hardware/avr/1.8.6

|

アップロード。

.. code-block:: console

    pi@zumo00:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino/AnalogMotor/AnalogMotor.ino

|

serial_motor.pyの作成

.. code-block:: console

    pi@zumo00:~$ cd ros2_ws/
    pi@zumo00:~/ros2_ws$ nano src/zm_test/zm_test/analog_motor.py

|

編集。

.. code-block:: python

    T.B.A.

|

setup.pyを開く。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ nano src/zm_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 29

    import os
    from glob import glob

    from setuptools import find_packages, setup

    package_name = 'zm_test'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*_launch.py')),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='ubuntu',
        maintainer_email='ubuntu@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'joy_led = zm_test.serial_led:main',
                'joy_motor = zm_test.serial_motor:main',
                'joy_analog_motor = zm_test.analog_motor:main',
            ],
        },
    )

|

ビルド。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ colcon build --packages-select zm_test
    Starting >>> zm_test 
    --- stderr: zm_test                    
    /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
    warnings.warn(
    ---
    Finished <<< zm_test [8.60s]

    Summary: 1 package finished [10.1s]
    1 package had stderr output: zm_test

|

セットアップファイルの反映。

.. code-block:: console

    pi@zumo00:~/ros2_ws$ source install/local_setup.bash

|

zm_testパッケージのjoy_motorノードの実行

.. code-block:: console

    pi@zumo00:~/ros2_ws$ ros2 run zm_test joy_analog_motor

|

joyパッケージのjoy_nodeの実行

.. code-block:: console

    ubuntu@mbc084:~$ ros2 run joy joy_node
