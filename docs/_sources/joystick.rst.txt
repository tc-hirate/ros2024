============================================================
ジョイスティックの使い方
============================================================

PCでジョイスティックが使えるように設定します。

|

必要なパッケージのインストール
============================================================

ジョイスティックを使うために必要なパッケージをインストールします。

.. code-block:: console

    ubuntu@mbc084:~$ sudo apt install ros-iron-joy

.. code-block:: console

    ubuntu@mbc084:~$ sudo apt install ros-iron-teleop-tools

|

ジョイスティックの動作確認
============================================================

PCにジョイスティックを接続し、次のコマンドを実行してください。

接続したジョイスティックが確認できます。（名前はF310かF150のどちらか）

.. code-block:: console

    ubuntu@mbc084:~$ lsusb
    ...
    Bus 002 Device 005: ID 046d:c21e Logitech, Inc. F510 Gamepad [XInput Mode]
    ...

|

続いて、次のコマンドを実行してください。

js0があればジョイスティックは認識されています。

.. code-block:: console

    ubuntu@mbc084:~$ ls /dev/input/
    by-id    event0  event10  event12  event2  event4  event6  event8  js0   mouse0
    by-path  event1  event11  event13  event3  event5  event7  event9  mice  mouse1

|

joyパッケージのjoy-nodeの実行。

.. code-block:: console

    ubuntu@mbc084:~$ ros2 run joy joy_node 
    [INFO] [1689135140.406199333] [joy_node]: Opened joystick: Logitech Gamepad F510.  deadzone: 0.050000

|

Topicの確認。

.. code-block:: console

    ubuntu@mbc084:~$ ros2 topic list
    /joy
    /joy/set_feedback
    /parameter_events
    /rosout

|

/joyの表示。

.. code-block:: console

    ubuntu@mbc084:~$ ros2 topic echo /joy
    header:
      stamp:
        sec: 1689135223
        nanosec: 171012534
      frame_id: joy
    axes:
    - -0.0
    - -0.0
    - 1.0
    - -0.0
    - -0.0
    - 1.0
    - 0.0
    - 0.0
    buttons:
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    ---

|

teleop_twist_joyパッケージのteleop-nodeの実行。

.. code-block:: console

    ubuntu@mbc084:~$ ros2 run teleop_twist_joy teleop_node 
    [INFO] [1689135379.152090618] [TeleopTwistJoy]: Teleop enable button 5.
    [INFO] [1689135379.152189267] [TeleopTwistJoy]: Linear axis x on 5 at scale 0.500000.
    [INFO] [1689135379.152221882] [TeleopTwistJoy]: Angular axis yaw on 2 at scale 0.500000.

|

Topicの確認。

.. code-block:: console

    ubuntu@mbc084:~$ ros2 topic list
    /cmd_vel
    /joy
    /joy/set_feedback
    /parameter_events
    /rosout

|

/cmd_velの表示。

RBを押しながらRTを操作するとxの値が-0.5〜0.5で変化し、LTを操作するとzの値が-0.5〜0.5で変化します。

.. code-block:: console

    ubuntu@mbc084:~$ ros2 topic echo /cmd_vel 
    linear:
      x: 0.5
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.5
    ---

|

パッケージの作成
============================================================

joy_testという名前のパッケージを作ります。

.. code-block:: console

    ubuntu@mbc084:~$ cd ros2_ws/src/
    ubuntu@mbc084:~/ros2_ws/src$ ros2 pkg create --build-type ament_python joy_test

|

launchファイル用のディレクトリを作ります。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws/src$ cd joy_test/
    ubuntu@mbc084:~/ros2_ws/src/joy_test$ mkdir launch

|

演習1「ジョイスティックを使ってturtlesimを動かす」
============================================================

|

（１）joy-nodeとteleop-nodeを使ってturtlesimを動かす
------------------------------------------------------------

turtlesimパッケージのturtlesim_nodeの実行。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws/src/joy_test$ ros2 run turtlesim turtlesim_node

|

joyパッケージのjoy-nodeの実行。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws/src/joy_test$ ros2 run joy joy_node

|

teleop_twist_joyパッケージのteleop_nodeの実行。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws/src/joy_test$ ros2 run teleop_twist_joy teleop_node --ros-args --remap /cmd_vel:=/turtle1/cmd_vel

|

（２）launchファイルの作成
------------------------------------------------------------

joy-nodeとteleop-nodeを使ってturtlesimを動かすためのlaunchファイルを作成してください。

ファイル名は「turtle_teleop_joy_launch.py」とします。

|

turtle_teleop_joy_launch.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws/src/joy_test$ cd launch/
    ubuntu@mbc084:~/ros2_ws/src/joy_test/launch$ nano turtle_teleop_joy_launch.py

|

編集。

.. code-block:: python

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                name="sim",
                package="turtlesim",
                executable="turtlesim_node",
            ),
            Node(
                name="joy",
                package="joy",
                executable="joy_node",
            ),
            Node(
                name="teleop",
                package="teleop_twist_joy",
                executable="teleop_node",
                remappings=[
                    ('/cmd_vel', '/turtle1/cmd_vel'),
                ],
            ),
        ])

|

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws/src/joy_test/launch$ cd ..
    ubuntu@mbc084:~/ros2_ws/src/joy_test$ nano setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 1, 2, 16

    import os
    from glob import glob

    from setuptools import find_packages, setup

    package_name = 'joy_test'

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
            ],
        },
    )

|

ワークスペースに移動。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws/src/joy_test$ cd ~/ros2_ws/

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select joy_test

|

setupファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/setup.bash

|

launchファイルの実行。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 launch joy_test turtle_teleop_joy_launch.py

|

ノードの確認。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 node list
    /joy
    /sim
    /teleop

|

rqt_graphでノードの確認。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ rqt_graph

|

.. image:: ./img/joystick_img_01.png
   :align: center

|

（３）ジョイスティックの方向キーでturtlesimを動かす
------------------------------------------------------------

ジョイスティックの方向キーでturtlesimを動かすプログラムを作ってください。

ファイル名は「turtle_joy.py」、仕様はturtle_teleop_keyと同じとします。

.. note::

   ここからはディレクトリを移動しないで「ros2_ws」からコマンドを実行します。

|

turtle_joy.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/joy_test/joy_test/turtle_joy.py

|

編集。

.. code-block:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy

    class JoyTwist(Node):

        def __init__(self):
            super().__init__('joy_twist')
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.subscription = self.create_subscription(
                Joy,
                'joy',
                self.joy_callback,
                10)
            self.subscription

        def joy_callback(self, joy_msg):
            twist = Twist()
            if joy_msg.axes[7] == 1:
                twist.linear.x = 2.0
            elif joy_msg.axes[7] == -1:
                twist.linear.x = -2.0
            elif joy_msg.axes[6] == 1:
                twist.angular.z = 2.0
            elif joy_msg.axes[6] == -1:
                twist.angular.z = -2.0
            else:
                twist.linear.x = 0.0
            self.publisher_.publish(twist)

    def main(args=None):
        rclpy.init(args=args)

        joy_twist = JoyTwist()

        rclpy.spin(joy_twist)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        joy_twist.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

|

package.xmlを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/joy_test/package.xml

|

編集。

.. code-block:: none
    :emphasize-lines: 10-13

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematyp>
    <package format="3">
      <name>joy_test</name>
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

    ubuntu@mbc084:~/ros2_ws$ nano src/joy_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 27

    import os
    from glob import glob

    from setuptools import find_packages, setup

    package_name = 'joy_test'

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
                'joy_twist = joy_test.turtle_joy:main',
            ],
        },
    )

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select joy_test

|

setupファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/setup.bash

|

turtlesimパッケージのturtlesim_nodeの実行。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run turtlesim turtlesim_node

|

joyパッケージのjoy-nodeの実行。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run joy joy_node

|

joy_testパッケージのjoy_twistノードの実行。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run joy_test joy_twist

|

（４）launchファイルの作成
------------------------------------------------------------

「turtle_joy.py」を実行するlaunchファイルを作成してください。

ファイル名は「turtle_joy_launch.py」とします。

|

turtle_joy_launch.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/joy_test/launch/turtle_joy_launch.py

|

編集。

.. code-block:: python

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                name='sim',
                package='turtlesim',
                executable='turtlesim_node',
            ),
            Node(
                name='joy',
                package='joy',
                executable='joy_node',
            ),
            Node(
                name='test',
                package='joy_test',
                executable='joy_twist',
            ),
    ])

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select joy_test

|

setupファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/setup.bash

|

launchファイルの実行。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 launch joy_test turtle_joy_launch.py

|
