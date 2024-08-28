.. role:: dir

============================================================
Arduinoの使い方
============================================================

Raspberry PiでArduinoを使えるように設定します。

Arduinoを使って、 LEDを点灯させたり、モーターを動かしたりします。 また、ROSと通信をする方法について説明します。

Arduinoのプログラムはスケッチと呼ばれ、C/C++がベースとなっています。 Pythonとは異なるので注意してください。

開発環境をインストールする
============================================================

arduino-cliをインストール。

.. code-block:: console

    pi@zumo01:~$ curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    Installing in /home/pi/bin
    ARCH=ARM64
    OS=Linux
    Using curl as download tool
    Downloading https://downloads.arduino.cc/arduino-cli/arduino-cli_1.0.3_Linux_ARM64.tar.gz
    install.sh: arduino-cli not found. You might want to add "/home/pi/bin" to your $PATH
    arduino-cli  Version: 1.0.3 Commit: 8b6ad258 Date: 2024-07-23T08:45:21Z installed successfully in /home/pi/bin

arduino-cliのパスを通す。

.. code-block:: console

    pi@zumo01:~$ export PATH=$PATH:$HOME/bin

arduino-cliのヘルプを表示して、パスが通っていることを確認。

.. code-block:: console

    pi@zumo01:~$ arduino-cli help core
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
          --json                      Print the output in JSON format.
          --log                       Print the logs on the standard output.
          --log-file string           Path to the file where logs will be written.
          --log-format string         The output format for the logs, can be: text, json (default "text")
          --log-level string          Messages with this level and above will be logged. Valid levels are: trace, debug, info, warn, error, fatal, panic (default "info")
          --no-color                  Disable colored output.

    Use "arduino-cli core [command] --help" for more information about a command.

パスを通すためのコマンドを shellのstartup scriptに追加。

.. code-block:: console

    pi@zumo01:~$ echo "export PATH=$PATH:$HOME/bin" >> ~/.bashrc

configuration fileを作成。

.. code-block:: console

    pi@zumo01:~$ arduino-cli config init
    Config file written to: /home/pi/.arduino15/arduino-cli.yaml

LEDを点滅させる
============================================================

Zumo Shieldには「user LED」があります。

Arduinoとは次のように接続されています。

.. csv-table::

    "ピン番号", "Zumo Shieldの機能"
    "13", "LED（LOW：消灯、HIGH：点灯）"

詳しくは「User's Guide」を確認してください。

この「user LED」を点滅させるスケッチを作ります。

スケッチの名前は「Led」とします。

|

次のコマンドを実行して、新しいスケッチを作成。

.. code-block:: console

    pi@zumo01:~$ arduino-cli sketch new Arduino/Led
    Sketch created in: /home/pi/Arduino/Led

Led.inoを開く。

.. code-block:: console

    pi@zumo01:~$ nano Arduino/Led/Led.ino

編集前。

.. code-block:: c
    :caption: Led.ino

    void setup() {
    }

    void loop() {
    }

編集。

.. code-block:: c
    :caption: Led.ino

    void setup() {
      pinMode(13, OUTPUT);
    }

    void loop() {
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
      delay(1000);
    }

setup()には、ピンをどのように設定するかを書きます。

LEDがデジタルピン13に接続されているので、ピン13を出力に設定します。

.. code-block:: c

    void setup() {
      pinMode(13, OUTPUT);
    }

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

利用できるプラットフォームとライブラリを更新。

.. code-block:: console

    pi@zumo01:~$ arduino-cli core update-index
    Downloading index: library_index.tar.bz2 downloaded              
    Downloading index: package_index.tar.bz2 downloaded              
    Downloading missing tool builtin:ctags@5.8-arduino11...
    builtin:ctags@5.8-arduino11 downloaded                           
    Installing builtin:ctags@5.8-arduino11...
    Skipping tool configuration....
    builtin:ctags@5.8-arduino11 installed
    Downloading missing tool builtin:dfu-discovery@0.1.2...
    builtin:dfu-discovery@0.1.2 downloaded                           
    Installing builtin:dfu-discovery@0.1.2...
    Skipping tool configuration....
    builtin:dfu-discovery@0.1.2 installed
    Downloading missing tool builtin:mdns-discovery@1.0.9...
    builtin:mdns-discovery@1.0.9 365.64 KiB / 2.19 MiB   16.33% 00m01builtin:mdns-discovery@1.0.9 downloaded                          
    Installing builtin:mdns-discovery@1.0.9...
    Skipping tool configuration....
    builtin:mdns-discovery@1.0.9 installed
    Downloading missing tool builtin:serial-discovery@1.4.1...
    builtin:serial-discovery@1.4.1 downloaded                        
    Installing builtin:serial-discovery@1.4.1...
    Skipping tool configuration....
    builtin:serial-discovery@1.4.1 installed
    Downloading missing tool builtin:serial-monitor@0.14.1...
    builtin:serial-monitor@0.14.1 109.64 KiB / 1.93 MiB    5.54% 00m0builtin:serial-monitor@0.14.1 269.64 KiB / 1.93 MiB   13.63% 00m0builtin:serial-monitor@0.14.1 573.64 KiB / 1.93 MiB   29.00% 00m0builtin:serial-monitor@0.14.1 downloaded                         
    Installing builtin:serial-monitor@0.14.1...
    Skipping tool configuration....
    builtin:serial-monitor@0.14.1 installed
    Downloading index: package_index.tar.bz2 downloaded 

ArduinoとRaspberry Piを接続し、正しく認識されているか確認。

.. code-block:: console

    pi@zumo01:~$ arduino-cli board list
    Port          Protocol Type              Board Name          FQBN                          Core
    /dev/ttyACM0  serial   Serial Port (USB) Arduino UNO R4 WiFi arduino:renesas_uno:unor4wifi arduino:renesas_uno
    /dev/ttyAMA10 serial   Serial Port       Unknown

arduino:renesas_unoのplatform coreをインストール。

.. code-block:: console

    pi@zumo01:~$ arduino-cli core install arduino:renesas_uno
    Tool builtin:dfu-discovery@0.1.2 already installed
    Downloading packages...
    arduino:arm-none-eabi-gcc@7-2017q4 4.85 MiB / 94.95 MiB    5.10% arduino:arm-none-eabi-gcc@7-2017q4 4.85 MiB / 94.95 MiB    5.10% arduino:arm-none-eabi-gcc@7-2017q4 9.96 MiB / 94.95 MiB   10.49% arduino:arm-none-eabi-gcc@7-2017q4 13.30 MiB / 94.95 MiB   14.01%arduino:arm-none-eabi-gcc@7-2017q4 16.43 MiB / 94.95 MiB   17.31%arduino:arm-none-eabi-gcc@7-2017q4 16.43 MiB / 94.95 MiB   17.31%arduino:arm-none-eabi-gcc@7-2017q4 19.11 MiB / 94.95 MiB   20.12%arduino:arm-none-eabi-gcc@7-2017q4 22.10 MiB / 94.95 MiB   23.27%arduino:arm-none-eabi-gcc@7-2017q4 26.19 MiB / 94.95 MiB   27.58%arduino:arm-none-eabi-gcc@7-2017q4 29.62 MiB / 94.95 MiB   31.20%arduino:arm-none-eabi-gcc@7-2017q4 29.62 MiB / 94.95 MiB   31.20%arduino:arm-none-eabi-gcc@7-2017q4 34.39 MiB / 94.95 MiB   36.22%arduino:arm-none-eabi-gcc@7-2017q4 38.17 MiB / 94.95 MiB   40.20%arduino:arm-none-eabi-gcc@7-2017q4 40.91 MiB / 94.95 MiB   43.09%arduino:arm-none-eabi-gcc@7-2017q4 45.48 MiB / 94.95 MiB   47.90%arduino:arm-none-eabi-gcc@7-2017q4 45.48 MiB / 94.95 MiB   47.90%arduino:arm-none-eabi-gcc@7-2017q4 47.63 MiB / 94.95 MiB   50.17%arduino:arm-none-eabi-gcc@7-2017q4 52.67 MiB / 94.95 MiB   55.47%arduino:arm-none-eabi-gcc@7-2017q4 56.30 MiB / 94.95 MiB   59.30%arduino:arm-none-eabi-gcc@7-2017q4 59.87 MiB / 94.95 MiB   63.05%arduino:arm-none-eabi-gcc@7-2017q4 59.87 MiB / 94.95 MiB   63.05%arduino:arm-none-eabi-gcc@7-2017q4 63.18 MiB / 94.95 MiB   66.54%arduino:arm-none-eabi-gcc@7-2017q4 67.49 MiB / 94.95 MiB   71.08%arduino:arm-none-eabi-gcc@7-2017q4 70.29 MiB / 94.95 MiB   74.03%arduino:arm-none-eabi-gcc@7-2017q4 73.35 MiB / 94.95 MiB   77.25%arduino:arm-none-eabi-gcc@7-2017q4 73.35 MiB / 94.95 MiB   77.25%arduino:arm-none-eabi-gcc@7-2017q4 78.44 MiB / 94.95 MiB   82.61%arduino:arm-none-eabi-gcc@7-2017q4 downloaded                    
    arduino:bossac@1.9.1-arduino5 downloaded                         
    arduino:dfu-util@0.11.0-arduino5 downloaded                      
    arduino:openocd@0.11.0-arduino2 downloaded                       
    arduino:renesas_uno@1.2.0 downloaded                             
    Installing arduino:arm-none-eabi-gcc@7-2017q4...
    Configuring tool....
    arduino:arm-none-eabi-gcc@7-2017q4 installed
    Installing arduino:bossac@1.9.1-arduino5...
    Configuring tool....
    arduino:bossac@1.9.1-arduino5 installed
    Installing arduino:dfu-util@0.11.0-arduino5...
    Configuring tool....
    arduino:dfu-util@0.11.0-arduino5 installed
    Installing arduino:openocd@0.11.0-arduino2...
    Configuring tool....
    arduino:openocd@0.11.0-arduino2 installed
    Installing platform arduino:renesas_uno@1.2.0...
    Configuring platform....
    Please run as root

    Platform arduino:renesas_uno@1.2.0 installed

正しくインストールされたか確認。

.. code-block:: console

    pi@zumo01:~$ arduino-cli core list
    ID                  Installed Latest Name
    arduino:renesas_uno 1.2.0     1.2.0  Arduino UNO R4 Boards
    arduino:renesas_uno:unor4wifi

コンパイル。

.. code-block:: console

    pi@zumo01:~$ arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Arduino/Led
    Sketch uses 52168 bytes (19%) of program storage space. Maximum is 262144 bytes.
    Global variables use 6744 bytes (20%) of dynamic memory, leaving 26024 bytes for local variables. Maximum is 32768 bytes.

    Used platform       Version Path
    arduino:renesas_uno 1.2.0   /home/pi/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0

アップロード。

.. code-block:: console

    pi@zumo01:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi Arduino/Led
    Erase flash

    Done in 0.002 seconds
    Write 52176 bytes to flash (13 pages)
    [==============================] 100% (13/13 pages)
    Done in 2.824 seconds
    New upload port: /dev/ttyACM0 (serial)

2秒周期でLEDが点滅（1秒点灯、1秒消灯）していることを確認してください。

|

押しボタンスイッチを使ってLEDを点灯させる
============================================================

Zumo Shieldには「user pushbutton」があります。

Arduinoとは次のように接続されています。

.. csv-table::

    "ピン番号", "Zumo Shieldの機能"
    "12", "押しボタンスイッチ（LOW：押されている、HIGH：押されていない）"

詳しくは「User's Guide」を確認してください。

この「user pushbutton」が押されているときに「user LED」を点灯し、押されていないときに消灯するスケッチを作ります。

スケッチの名前は「Button」とします。

|

スケッチの作成。

.. code-block:: console

    pi@zumo01:~$ arduino-cli sketch new Arduino/Button
    Sketch created in: /home/pi/Arduino/Button

Button.inoを開く。

.. code-block:: console

    pi@zumo01:~$ nano Arduino/Button/Button.ino

編集。

.. code-block:: c
    :caption: Button.ino

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

pushbuttonが押されているかどうかを記憶しておくための変数を定義しています。

.. code-block:: c

    int val = 0;

pushbuttonがデジタルピン12に接続されているので、ピン12を入力、プルアップ抵抗を有効に設定しています。

.. code-block:: c

    pinMode(12, INPUT_PULLUP);

pushbuttonが押されているかどうかを読み込んでいます。

.. code-block:: c

    val = digitalRead(12);

コンパイル。

.. code-block:: console

    pi@zumo01:~$ arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Arduino/Button/
    Sketch uses 52248 bytes (19%) of program storage space. Maximum is 262144 bytes.
    Global variables use 6748 bytes (20%) of dynamic memory, leaving 26020 bytes for local variables. Maximum is 32768 bytes.

    Used platform       Version Path
    arduino:renesas_uno 1.2.0   /home/pi/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0

アップロード。

.. code-block:: console

    pi@zumo01:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi Arduino/Button/
    Erase flash

    Done in 0.002 seconds
    Write 52256 bytes to flash (13 pages)
    [==============================] 100% (13/13 pages)
    Done in 2.823 seconds
    New upload port: /dev/ttyACM0 (serial)

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

詳しくは「User's Guide」を確認してください。

「user pushbutton」が押されているときに前進し、押されていないときに停止するスケッチを作ります。

スケッチの名前は「Motor」とします。

|

スケッチの作成。

.. code-block:: console

    pi@zumo01:~$ arduino-cli sketch new Arduino/Motor
    Sketch created in: /home/pi/Arduino/Motor

Motor.inoを開く。

.. code-block:: console

    pi@zumo01:~$ nano Arduino/Motor/Motor.ino

編集。

.. code-block:: c
    :caption: Motor.ino

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

前進するときは各モータの方向制御にLOWを入力します。

.. code-block:: c

    digitalWrite(DIRECTION_R, LOW);
    digitalWrite(DIRECTION_L, LOW);

PWM出力するときはanalogWrite(ピン番号, PWM値)を使います。PWM値には0〜100を設定します。

.. code-block:: c

    analogWrite(PWM_R, 30);
    analogWrite(PWM_L, 30);

コンパイル。

.. code-block:: console

    pi@zumo01:~$ arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Arduino/Motor/
    Sketch uses 54668 bytes (20%) of program storage space. Maximum is 262144 bytes.
    Global variables use 6752 bytes (20%) of dynamic memory, leaving 26016 bytes for local variables. Maximum is 32768 bytes.

    Used platform       Version Path
    arduino:renesas_uno 1.2.0   /home/pi/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0

アップロード。

.. code-block:: console

    pi@zumo01:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi Arduino/Motor/
    Erase flash

    Done in 0.002 seconds
    Write 54676 bytes to flash (14 pages)
    [==============================] 100% (14/14 pages)
    Done in 3.041 seconds
    New upload port: /dev/ttyACM0 (serial)

「user pushbutton」を押したときにモーターが動くことを確認してください。

|

Arduinoと通信する
============================================================

ROS1では、rosserialというツールを使ってtopicを送受信することができました。

ROS2にもros2arduinoというツールがあるのですが、Auduino UNOには対応していないようです。

.. note::

    ESP32だとmicro-rosが使える。
    Arduinoでも使えるようにならないか。
   
|

ここでは、Pythonでシリアル通信をするためのモジュールpyserialを使います。

pyserialはpipを使ってインストールします。pipはPythonのパッケージを管理するためのツールです。

.. code-block:: console

    pi@zumo01:~$ sudo apt install python3-pip

pipでpyserialをインストールしようとするとエラーが発生。

.. code-block:: console

    pi@zumo01:~$ python3 -m pip install pyserial
    error: externally-managed-environment

    × This environment is externally managed
    ╰─> To install Python packages system-wide, try apt install
        python3-xyz, where xyz is the package you are trying to
        install.
        
        If you wish to install a non-Debian-packaged Python package,
        create a virtual environment using python3 -m venv path/to/venv.
        Then use path/to/venv/bin/python and path/to/venv/bin/pip. Make
        sure you have python3-full installed.
        
        If you wish to install a non-Debian packaged Python application,
        it may be easiest to use pipx install xyz, which will manage a
        virtual environment for you. Make sure you have pipx installed.
        
        See /usr/share/doc/python3.12/README.venv for more information.

    note: If you believe this is a mistake, please contact your Python installation or OS distribution provider. You can override this, at the risk of breaking your Python installation or OS, by passing --break-system-packages.
    hint: See PEP 668 for the detailed specification.

エラーを回避するために「--break-system-packages」というオプションをつけてインストール。

.. code-block:: console

    pi@zumo01:~$ python3 -m pip install --break-system-packages pyserial
    Defaulting to user installation because normal site-packages is not writeable
    WARNING: Skipping /usr/lib/python3.12/dist-packages/argcomplete-3.1.4.dist-info due to invalid metadata entry 'name'
    Requirement already satisfied: pyserial in /usr/lib/python3/dist-packages (3.5)
    WARNING: Skipping /usr/lib/python3.12/dist-packages/argcomplete-3.1.4.dist-info due to invalid metadata entry 'name'

|

ターミナルから0または1を入力して、0ならばLEDを消灯、1ならばLEDを点灯するプログラムを作ります。

Pythonのプログラムを保存するために、ホームディレクトリにPythonという名前の ディレクトリを作成。

.. code-block:: console

    pi@zumo01:~$ mkdir Python

ここにserial_test.pyという名前でファイルを作成。

.. code-block:: console

    pi@zumo01:~$ nano Python/serial_test.py

編集。

.. code-block:: python
    :caption: serial_test.py

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

試しに実行。

.. code-block:: console

    pi@zumo01:~$ python3 Python/serial_test.py
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

    pi@zumo01:~$ arduino-cli sketch new Arduino/SerialTest
    Sketch created in: /home/pi/Arduino/SerialTest

SerialTest.inoを開く。

.. code-block:: console

    pi@zumo01:~$ nano Arduino/SerialTest/SerialTest.ino

編集。

.. code-block:: c
    :caption: SerialTest.ino

    const int LED = 13;

    void setup() {
      Serial.begin(9600);
      pinMode(LED, OUTPUT);
      digitalWrite(LED, LOW);
    }

    void loop() {
      byte var;
      var = Serial.read();
      switch(var) {
        case '0':
          digitalWrite(LED, LOW);
          break;
        case '1':
          digitalWrite(LED, HIGH);
          break;
        default:
          break;
      }
    }

コンパイル。

.. code-block:: console

    pi@zumo01:~$ arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Arduino/SerialTest/
    Sketch uses 52200 bytes (19%) of program storage space. Maximum is 262144 bytes.
    Global variables use 6744 bytes (20%) of dynamic memory, leaving 26024 bytes for local variables. Maximum is 32768 bytes.

    Used platform       Version Path
    arduino:renesas_uno 1.2.0   /home/pi/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0

アップロード。

.. code-block:: console

    pi@zumo01:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi Arduino/SerialTest/
    Erase flash

    Done in 0.002 seconds
    Write 52208 bytes to flash (13 pages)
    [==============================] 100% (13/13 pages)
    Done in 2.825 seconds
    New upload port: /dev/ttyACM0 (serial)

ターミナルから入力した数値に応じてLEDが点灯／消灯するか確認してください。

|

演習1「ジョイスティックのAボタンを使ってLEDを点灯させる」
============================================================

ジョイスティックのAボタンを押すとLEDが点灯するプログラムを作ってください。

Pythonのプログラムは、zm_testパッケージの「serial_led.py」とします。

また、スケッチは「12.5. Arduinoと通信する」で作った「SerialTest」を使用します。

|

ワークスペースの作成。

.. code-block:: console

    pi@zumo01:~$ mkdir -p ~/ros2_ws/src

|

ワークスペースのsrcディレクトリへ移動。

.. code-block:: console

    pi@zumo01:~$ cd ~/ros2_ws/src/

パッケージの作成。

.. code-block:: console

    pi@zumo01:~/ros2_ws/src$ ros2 pkg create --build-type ament_python zm_test
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

ワークスペースへ移動。

.. code-block:: console

    pi@zumo01:~/ros2_ws/src$ cd ..

serial_led.pyの作成。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ nano src/zm_test/zm_test/serial_led.py

編集。

.. .. code-block:: python
..     :caption: serial_led.py

..     import rclpy
..     from rclpy.node import Node

..     from std_msgs.msg import String
..     from geometry_msgs.msg import Twist
..     from sensor_msgs.msg import Joy

..     import serial

..     class JoyLed(Node):

..         def __init__(self):
..             super().__init__('joy_led')
..             self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
..             self.subscription = self.create_subscription(
..                 Joy,
..                 'joy',
..                 self.joy_callback,
..                 10)
..             self.subscription

..             self.get_logger().info('Open Port')
..             self.ser = serial.Serial()
..             self.ser.port = "/dev/ttyACM0"
..             self.ser.baudrate = 9600
..             self.ser.open()

..         def joy_callback(self, joy_msg):
..             twist = Twist()
..             # your code

..             self.publisher_.publish(twist)

..     def main(args=None):
..         rclpy.init(args=args)

..         joy_led = JoyLed()

..         rclpy.spin(joy_led)

..         # Destroy the node explicitly
..         # (optional - otherwise it will be done automatically
..         # when the garbage collector destroys the node object)
..         joy_led.destroy_node()
..         rclpy.shutdown()

..     if __name__ == '__main__':
..         main()

.. code-block:: python
    :caption: serial_led.py

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

package.xmlを開く。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ nano src/zm_test/package.xml

編集。

.. code-block:: none
    :emphasize-lines: 10-13
    :caption: package.xml

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

setup.pyを開く。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ nano src/zm_test/setup.py

編集。

.. code-block:: python
    :emphasize-lines: 23
    :caption: setup.py

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
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='pi',
        maintainer_email='pi@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'joy_led = zm_test.serial_led:main',
            ],
        },
    )

コルコンのインストール。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ sudo apt install python3-colcon-common-extensions

ビルド。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ colcon build --packages-select zm_test
    Starting >>> zm_test 
    Finished <<< zm_test [2.06s]          

    Summary: 1 package finished [2.30s]

セットアップファイルの反映。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ source install/local_setup.bash

zm_testパッケージのjoy_ledノードの実行

.. code-block:: console

    pi@zumo01:~/ros2_ws$ ros2 run zm_test joy_led

joyパッケージのjoy_nodeの実行

.. code-block:: console

    ubuntu@mbc112:~$ ros2 run joy joy_node

|

演習2「ジョイスティックの方向キーでzumoを動かす」
============================================================

ジョイスティックの方向キーでZumoを操縦するプログラムを作ってください。

Pythonのプログラムは、zm_testパッケージの「serial_motor.py」とします。

また、スケッチの名前は「SerialMotor」とします。

|

ホームディレクトリへ移動。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ cd

スケッチの作成。

.. code-block:: console

    pi@zumo01:~$ arduino-cli sketch new Arduino/SerialMotor
    Sketch created in: /home/pi/Arduino/SerialMotor

SerialMotor.inoを開く。

.. code-block:: console

    pi@zumo01:~$ nano Arduino/SerialMotor/SerialMotor.ino

編集。

.. code-block:: c
    :caption: SerialMotor.ino

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

コンパイル。

.. code-block:: console

    pi@zumo01:~$ arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Arduino/SerialMotor/
    Sketch uses 54716 bytes (20%) of program storage space. Maximum is 262144 bytes.
    Global variables use 6752 bytes (20%) of dynamic memory, leaving 26016 bytes for local variables. Maximum is 32768 bytes.

    Used platform       Version Path
    arduino:renesas_uno 1.2.0   /home/pi/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0

アップロード。

.. code-block:: console

    pi@zumo01:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi Arduino/SerialMotor/
    Erase flash

    Done in 0.002 seconds
    Write 54724 bytes to flash (14 pages)
    [==============================] 100% (14/14 pages)
    Done in 3.041 seconds
    New upload port: /dev/ttyACM0 (serial)

ワークスペースへ移動。

.. code-block:: console

    pi@zumo01:~$ cd ros2_ws/

serial_motor.pyの作成。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ nano src/zm_test/zm_test/serial_motor.py

編集。

.. code-block:: python
    :caption: serial_motor.py

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
            # your code

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

.. .. code-block:: python
..     :caption: serial_motor.py

..     import rclpy
..     from rclpy.node import Node

..     from std_msgs.msg import String
..     from geometry_msgs.msg import Twist
..     from sensor_msgs.msg import Joy

..     import serial

..     class JoyMotor(Node):

..         def __init__(self):
..             super().__init__('joy_motor')
..             self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
..             self.subscription = self.create_subscription(
..                 Joy,
..                 'joy',
..                 self.joy_callback,
..                 10)
..             self.subscription

..             self.get_logger().info('Open Port')
..             self.ser = serial.Serial()
..             self.ser.port = "/dev/ttyACM0"
..             self.ser.baudrate = 9600
..             self.ser.open()

..         def __del__(self):
..             self.get_logger().info('Close Port')

..         def joy_callback(self, joy_msg):
..             twist = Twist()
..             if joy_msg.axes[7] == 1:  # 上が押されたら前進
..                 self.get_logger().info('Forward')
..                 self.ser.write(b"1")
..             elif joy_msg.axes[7] == -1:  # 下が押されたら後進
..                 self.get_logger().info('Backward')
..                 self.ser.write(b"2")
..             elif joy_msg.axes[6] == 1:  # 左が押されたら左に曲がる
..                 self.get_logger().info('Left')
..                 self.ser.write(b"3")
..             elif joy_msg.axes[6] == -1:  # 右が押されたら右に曲がる
..                 self.get_logger().info('Right')
..                 self.ser.write(b"4")
..             else:  # それ以外のときは停止
..                 self.get_logger().info('Stop')
..                 self.ser.write(b"0")
..             self.publisher_.publish(twist)

..     def main(args=None):
..         rclpy.init(args=args)

..         joy_motor = JoyMotor()

..         rclpy.spin(joy_motor)

..         # Destroy the node explicitly
..         # (optional - otherwise it will be done automatically
..         # when the garbage collector destroys the node object)
..         self.get_logger().info('Close Port')
..         joy_motor.destroy_node()
..         rclpy.shutdown()


..     if __name__ == '__main__':
..         main()

setup.pyを開く。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ nano src/zm_test/setup.py

編集。

.. code-block:: python
    :emphasize-lines: 24
    :caption: setup.py

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
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='pi',
        maintainer_email='pi@todo.todo',
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

ビルド。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ colcon build --packages-select zm_test
    Starting >>> zm_test 
    Finished <<< zm_test [2.22s]          

    Summary: 1 package finished [2.40s]

セットアップファイルの反映。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ source install/local_setup.bash

zm_testパッケージのjoy_motorノードの実行

.. code-block:: console

    pi@zumo01:~/ros2_ws$ ros2 run zm_test joy_motor

joyパッケージのjoy_nodeの実行

.. code-block:: console

    ubuntu@mbc112:~$ ros2 run joy joy_node

|

演習3「ジョイスティックのアナログスイッチでzumoを動かす」
============================================================

ジョイスティックの左アナログスイッチでZumoを操縦するプログラムを作ってください。 

Pythonのプログラムは、zm_testパッケージの「analog_motor.py」とします。

また、スケッチの名前は「AnalogMotor」とします。

.. note::

    下のプログラムは一応動くけど、
    他に良い方法があれば教えてください。
   
|

ホームディレクトリへ移動。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ cd

スケッチの作成。

.. code-block:: console

    pi@zumo01:~$ arduino-cli sketch new Arduino/AnalogMotor
    Sketch created in: /home/pi/Arduino/AnalogMotor

AnalogMotor.inoを開く。

.. code-block:: console

    pi@zumo01:~$ nano Arduino/AnalogMotor/AnalogMotor.ino

編集。

.. code-block:: c
    :caption: AnalogMotor.ino

    const int DIRECTION_R = 7;
    const int DIRECTION_L = 8;
    const int PWM_R = 9;
    const int PWM_L = 10;

    const int LED = 13;
    const int BUTTON = 12;

    int input = -1;
    int add = 0;
    int val0 = 0;
    int val1 = 0;
    int sign_flag = 1;

    int pwm_r = 0;
    int pwm_l = 0;

    void setup() {
      Serial.begin(9600);

      pinMode(DIRECTION_R, OUTPUT);
      pinMode(DIRECTION_L, OUTPUT);
      pinMode(PWM_R, OUTPUT);
      pinMode(PWM_L, OUTPUT);

      pinMode(LED, OUTPUT);
      digitalWrite(LED, LOW);
      pinMode(BUTTON, INPUT_PULLUP);
    }

    void loop() {
      input = Serial.read();

      if(input != -1) {
        switch(input) {
          case '0':
            add = 0 + add * 10;
            break;
          case '1':
            add = 1 + add * 10;
            break;
          case '2':
            add = 2 + add * 10;
            break;
          case '3':
            add = 3 + add * 10;
            break;
          case '4':
            add = 4 + add * 10;
            break;
          case '5':
            add = 5 + add * 10;
            break;
          case '6':
            add = 6 + add * 10;
            break;
          case '7':
            add = 7 + add * 10;
            break;
          case '8':
            add = 8 + add * 10;
            break;
          case '9':
            add = 9 + add * 10;
            break;
          case '-':
            sign_flag = -1;
            break;
          case ',':
            val0 = add * sign_flag;
            add = 0;
            sign_flag = 1;
            break;
          case ';':
            val1 = add * sign_flag;
            add = 0;
            sign_flag = 1;
            break;
          default:
            break;
        }
      }

      if (val1 > 0) {
        pwm_r = (int)((val1 + val0) * 0.5);
        pwm_l = (int)((val1 - val0) * 0.5);
      }
      else {
        pwm_r = (int)((val1 - val0) * 0.5);
        pwm_l = (int)((val1 + val0) * 0.5);
      }

      if (pwm_r > 0) {
        digitalWrite(DIRECTION_R, LOW);
      }
      else {
        digitalWrite(DIRECTION_R, HIGH);
        pwm_r = abs(pwm_r);
      }

      if (pwm_l > 0) {
        digitalWrite(DIRECTION_L, LOW);
      }
      else {
        digitalWrite(DIRECTION_L, HIGH);
        pwm_l = abs(pwm_l);
      }

      analogWrite(PWM_R, pwm_r);
      analogWrite(PWM_L, pwm_l);
    }

コンパイル。

.. code-block:: console

    pi@zumo01:~$ arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi Arduino/AnalogMotor/
    Sketch uses 54940 bytes (20%) of program storage space. Maximum is 262144 bytes.
    Global variables use 6776 bytes (20%) of dynamic memory, leaving 25992 bytes for local variables. Maximum is 32768 bytes.

    Used platform       Version Path
    arduino:renesas_uno 1.2.0   /home/pi/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0

アップロード。

.. code-block:: console

    pi@zumo01:~$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi Arduino/AnalogMotor/
    Erase flash

    Done in 0.002 seconds
    Write 54948 bytes to flash (14 pages)
    [==============================] 100% (14/14 pages)
    Done in 3.042 seconds
    New upload port: /dev/ttyACM0 (serial)

ワークスペースへ移動。

.. code-block:: console

    pi@zumo01:~$ cd ros2_ws/

serial_motor.pyの作成

.. code-block:: console

    pi@zumo01:~/ros2_ws$ nano src/zm_test/zm_test/analog_motor.py

編集。

.. code-block:: python
    :caption: analog_motor.py

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy

    import serial

    class JoyLed(Node):

        def __init__(self):
            super().__init__('joy_led')
            self.publisher_ = self.create_publisher(Twist, '/turtle1>
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
            val0 = (int)(joy_msg.axes[0] * 100)
            val1 = (int)(joy_msg.axes[1] * 100)
            data = str(val0) + ',' + str(val1) + ';'
            self.get_logger().info(data)
            self.ser.write(bytes(data, 'utf-8'))

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

setup.pyを開く。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ nano src/zm_test/setup.py

編集。

.. code-block:: python
    :emphasize-lines: 25
    :caption: setup.py

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
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='pi',
        maintainer_email='pi@todo.todo',
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

ビルド。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ colcon build --packages-select zm_test
    Starting >>> zm_test 
    Finished <<< zm_test [1.73s]          

    Summary: 1 package finished [1.91s]

セットアップファイルの反映。

.. code-block:: console

    pi@zumo01:~/ros2_ws$ source install/local_setup.bash

zm_testパッケージのjoy_motorノードの実行

.. code-block:: console

    pi@zumo01:~/ros2_ws$ ros2 run zm_test joy_analog_motor

joyパッケージのjoy_nodeの実行

.. code-block:: console

    ubuntu@mbc112:~$ ros2 run joy joy_node
