============================================================
ROS2のインストール（Raspberry Pi）
============================================================

Raspberry PiにROS2をインストールします。

|

ROS2のインストール
============================================================

`ROS2のHP <https://docs.ros.org/en/iron/Installation.html>`_ の手順に従ってインストールします。

手順はPCと同じです。

|

ロケールの確認。

.. code-block:: console

    pi@zumo00:~$ locale
    LANG=C.UTF-8
    LANGUAGE=
    LC_CTYPE="C.UTF-8"
    LC_NUMERIC="C.UTF-8"
    LC_TIME="C.UTF-8"
    LC_COLLATE="C.UTF-8"
    LC_MONETARY="C.UTF-8"
    LC_MESSAGES="C.UTF-8"
    LC_PAPER="C.UTF-8"
    LC_NAME="C.UTF-8"
    LC_ADDRESS="C.UTF-8"
    LC_TELEPHONE="C.UTF-8"
    LC_MEASUREMENT="C.UTF-8"
    LC_IDENTIFICATION="C.UTF-8"
    LC_ALL=

|

Universeレポジトリを追加。

.. code-block:: console

    pi@zumo00:~$ sudo apt install software-properties-common
    pi@zumo00:~$ sudo add-apt-repository universe

|

ROS 2 GPG(GNU Privacy Guard) keyの追加。

.. code-block:: console

    pi@zumo00:~$ sudo apt update
    pi@zumo00:~$ sudo apt install curl -y
    pi@zumo00:~$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

|

レポジトリ情報をsource listに追加。

.. code-block:: console

    pi@zumo00:~$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

|

ROS2のインストール。

.. code-block:: console

    pi@zumo00:~$ sudo apt update
    pi@zumo00:~$ sudo apt install ros-iron-ros-base

.. warning::

   upgradeすると「Unable to locate package ros-iron-ros-base」というエラーが出る。

.. note::

   Raspberry PiにはROS-Baseをインストールします。

|

インストールの途中で次のようなウィンドウが出てきたら、

.. image:: ./img/ros_install_pi_img_01.png
   :align: center

|

［OK］を選択。

.. image:: ./img/ros_install_pi_img_02.png
   :align: center

|

サンプルプログラムの実行
============================================================

インストールが正しく行われたか確認するために、サンプルプログラムを実行します。

はじめに、setup fileを実行するコマンドをshellのstartup scriptに書いておきます。

.. code-block:: console

    pi@zumo00:~$ echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

|

続いて、IDを設定するコマンドもshellのstartup scriptに書いておきます。

IDはZumoの番号と同じにし、Zumo-00を使っている場合は0となります。

.. code-block:: console

    pi@zumo00:~$ echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

|

PCで次のコマンドを実行してください。

.. code-block:: console

    ubuntu@mbc084:~$ ros2 run demo_nodes_cpp talker

|

Raspberry Piで次のコマンドを実行してください。

.. code-block:: console

    pi@zumo00:~$ ros2 topic list
    /chatter
    /parameter_events
    /rosout

.. code-block:: console

    pi@zumo00:~$ ros2 topic echo /chatter
    data: 'Hello World: 39'
    ---
    data: 'Hello World: 40'
    ---
    data: 'Hello World: 41'
    ---
    data: 'Hello World: 42'
    ---
