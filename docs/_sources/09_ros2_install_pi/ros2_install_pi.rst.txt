============================================================
ROS2のインストール（Raspberry Pi）
============================================================

Raspberry PiにROS2をインストールします。

ROS2のインストール
============================================================

`ROS2のHP <https://docs.ros.org/en/jazzy/Installation.html>`_ の手順に従ってインストールします。

手順はPCと同じです。

|

ロケールの確認。

.. code-block:: console

    pi@zumo01:~$ locale
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

Universeレポジトリを追加。

.. code-block:: console

    pi@zumo01:~$ sudo apt install software-properties-common

.. code-block:: console

    pi@zumo01:~$ sudo add-apt-repository universe

ROS 2 GPG(GNU Privacy Guard) keyの追加。

.. code-block:: console

    pi@zumo01:~$ sudo apt update

.. code-block:: console

    pi@zumo01:~$ sudo apt install curl

.. code-block:: console

    pi@zumo01:~$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

レポジトリ情報をsource listに追加。

.. code-block:: console

    pi@zumo01:~$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

ROS2のインストール。

.. code-block:: console

    pi@zumo01:~$ sudo apt update

.. code-block:: console

    pi@zumo01:~$ sudo apt upgrade

.. code-block:: console

    pi@zumo01:~$ sudo apt install ros-jazzy-ros-base

.. note::

   Raspberry PiにはROS-Baseをインストールします。

サンプルプログラムの実行
============================================================

インストールが正しく行われたか確認するために、サンプルプログラムを実行します。

setup fileの実行。

.. code-block:: console

    pi@zumo01:~$ source /opt/ros/jazzy/setup.bash


ROS_DOMAIN_IDの設定。

.. code-block:: console

    pi@zumo01:~$ export ROS_DOMAIN_ID=1

PCで次のコマンドを実行。

.. code-block:: console

    ubuntu@mbc112:~$ ros2 run demo_nodes_cpp talker

Raspberry Piで次のコマンドを実行。

.. code-block:: console

    pi@zumo01:~$ ros2 topic list
    /chatter
    /parameter_events
    /rosout

Raspberry Piで次のコマンドを実行。

.. code-block:: console

    pi@zumo01:~$ ros2 topic echo /chatter 
    data: 'Hello World: 35'
    ---
    data: 'Hello World: 36'
    ---
    data: 'Hello World: 37'
    ---
    data: 'Hello World: 38'
    ---
    data: 'Hello World: 39'
    ---
    data: 'Hello World: 40'
    ---

shellのstartup scriptの追加。

.. code-block:: console

    pi@zumo00:~$ echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

.. code-block:: console

    pi@zumo00:~$ echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
