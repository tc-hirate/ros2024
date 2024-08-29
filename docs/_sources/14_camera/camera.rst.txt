============================================================
カメラの使い方
============================================================

ノートPCでは内蔵のカメラを、Raspberry PiではWebカメラを使用できるようにします。

|

必要なパッケージのインストール
============================================================

v4l2-cameraをインストール。

.. code-block:: console

    ubuntu@mbc112:~$ sudo apt-get install ros-jazzy-v4l2-camera

|

カメラの動作確認
============================================================

カメラが認識されているか確認。/dev/video0が表示されていていればOK。

.. code-block:: console

    ubuntu@mbc112:~$ ls /dev/video*
    /dev/video0  /dev/video1

v4l2_camera_nodeの実行。ERRORやWARNが出るが気にしない。

.. code-block:: console

    ubuntu@mbc112:~$ ros2 run v4l2_camera v4l2_camera_node
    [INFO] [1722820950.278395914] [v4l2_camera]: Driver: uvcvideo
    [INFO] [1722820950.279295594] [v4l2_camera]: Version: 395272
    [INFO] [1722820950.279446354] [v4l2_camera]: Device: Laptop_Integrated_Webcam_E4HD: 
    [INFO] [1722820950.279564235] [v4l2_camera]: Location: usb-0000:00:1a.0-1.5
    [INFO] [1722820950.279660733] [v4l2_camera]: Capabilities:
    [INFO] [1722820950.279741726] [v4l2_camera]:   Read/write: NO
    [INFO] [1722820950.279822749] [v4l2_camera]:   Streaming: YES
    [INFO] [1722820950.279968998] [v4l2_camera]: Current pixel format: YUYV @ 640x480
    [INFO] [1722820950.280239623] [v4l2_camera]: Available pixel formats: 
    [INFO] [1722820950.280300356] [v4l2_camera]:   YUYV - YUYV 4:2:2
    [INFO] [1722820950.280340087] [v4l2_camera]:   MJPG - Motion-JPEG
    [INFO] [1722820950.280378382] [v4l2_camera]: Available controls: 
    [INFO] [1722820950.280430428] [v4l2_camera]:   Brightness (1) = 0
    [INFO] [1722820950.280484977] [v4l2_camera]:   Contrast (1) = 0
    [INFO] [1722820950.280537938] [v4l2_camera]:   Saturation (1) = 64
    [INFO] [1722820950.281556245] [v4l2_camera]:   Hue (1) = 0
    [INFO] [1722820950.282664554] [v4l2_camera]:   White Balance, Automatic (2) = 1
    [INFO] [1722820950.282751183] [v4l2_camera]:   Gamma (1) = 100
    [INFO] [1722820950.282807379] [v4l2_camera]:   Power Line Frequency (3) = 2
    [INFO] [1722820950.284035461] [v4l2_camera]:   White Balance Temperature (1) = 4600 [inactive]
    [INFO] [1722820950.284116277] [v4l2_camera]:   Sharpness (1) = 2
    [INFO] [1722820950.284178199] [v4l2_camera]:   Backlight Compensation (1) = 3
    [INFO] [1722820950.289363242] [v4l2_camera]: Starting camera
    [WARN] [1722820950.898062774] [v4l2_camera]: Image encoding not the same as requested output, performing possibly slow conversion: yuv422_yuy2 => rgb8
    [INFO] [1722820950.905956512] [v4l2_camera]: using default calibration URL
    [INFO] [1722820950.906033505] [v4l2_camera]: camera calibration URL: file:///home/ubuntu/.ros/camera_info/laptop_integrated_webcam_e4hd:_.yaml
    [ERROR] [1722820950.906127106] [camera_calibration_parsers]: Unable to open camera calibration file [/home/ubuntu/.ros/camera_info/laptop_integrated_webcam_e4hd:_.yaml]
    [WARN] [1722820950.906194530] [v4l2_camera]: Camera calibration file /home/ubuntu/.ros/camera_info/laptop_integrated_webcam_e4hd:_.yaml not found

トピックの確認。/image_rawがカメラからのデータ。

.. code-block:: console

    ubuntu@mbc112:~$ ros2 topic list
    /camera_info
    /image_raw
    /parameter_events
    /rosout

rqt_image_viewで画像を表示。

.. code-block:: console

    ubuntu@mbc112:~$ ros2 run rqt_image_view rqt_image_view

|

.. image:: ./images/camera_img_01.png

|

画像処理をするプログラムを作る
============================================================

カメラで取得した画像をグレースケールに変換して表示してみましょう。

パケージはcv_test、ファイル名はcam_gray.pyとします。

画像処理は次の手順で行っています。ラズパイカメラから取得した画像はROSのImage型であることに注意してください。

- Webカメラの画像を取得する（data）
- OpenCVの標準データ形式に変換する（cv_image）
- 画像処理をする（cv_gray_image）
- ROSのImage型に変換する（ros_image）

|

ワークスペースへ移動。

.. code-block:: console

    ubuntu@mbc112:~$ cd ~/ros2_ws/

gray.pyをコピーしてcam_gray.pyを作成。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ cp src/cv_test/cv_test/gray.py src/cv_test/cv_test/cam_gray.py

cam_gray.pyを開く。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ nano src/cv_test/cv_test/cam_gray.py

編集。

.. code-block:: python
    :emphasize-lines: 11-17, 20-21
    :caption: cam_gray.py

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'gray_image', 10)
            self.subscription = self.create_subscription(
                Image,
                'image_raw',
                self.camera_callback,
                10)
            self.subscription
            self.bridge = CvBridge()

    def camera_callback(self, data):
            cv_image = self.bridge.imgmsg_to_cv2(data)
            cv_gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            ros_image = self.bridge.cv2_to_imgmsg(cv_gray_image, 'mono8')
            self.publisher_.publish(ros_image)


    def main(args=None):
        rclpy.init(args=args)

        image_publisher = ImagePublisher()

        rclpy.spin(image_publisher)

        image_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ nano src/cv_test/setup.py

編集。

.. code-block:: python
    :emphasize-lines: 30
    :caption: setup.py

    from setuptools import find_packages, setup

    package_name = 'cv_test'

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
        maintainer='ubuntu',
        maintainer_email='ubuntu@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'img_publisher = cv_test.image_view:main',
                'gray_publisher = cv_test.gray:main',
                'circle_publisher = cv_test.circle:main',
                'binary_publisher = cv_test.binary:main',
                'edge_publisher = cv_test.edge:main',
                'face_publisher = cv_test.face_detect:main',
                'eye_publisher = cv_test.eye_detect:main',
                'cam_publisher = cv_test.cam_gray:main',
            ],
        },
    )

ビルド。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ colcon build --packages-select cv_test
    Starting >>> cv_test 
    Finished <<< cv_test [2.26s]          

    Summary: 1 package finished [2.54s]

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ source install/local_setup.bash

cv_testパッケージのcam_publisherノードの実行

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ ros2 run cv_test cam_publisher

v4l2_camera_nodeの実行。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ ros2 run v4l2_camera v4l2_camera_node

rqt_image_viewで画像を表示。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ ros2 run rqt_image_view rqt_image_view

|

.. image:: ./images/camera_img_02.png

|

演習1「カメラの画像を使って、顔認識するプログラムを作ってください」
===================================================================

ファイル名は「cam_face_detect.py」とします。

|

face_detect.pyをコピーしてcam_face_detect.pyを作成。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ cp src/cv_test/cv_test/face_detect.py src/cv_test/cv_test/cam_face_detect.py

cam_face_detect.pyを開く。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ nano src/cv_test/cv_test/cam_face_detect.py

編集。

.. code-block:: python
    :emphasize-lines: 11-17, 20-22 
    :caption: cam_face_detect.py

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'face_detect', 10)
            self.subscription = self.create_subscription(
                Image,
                'image_raw',
                self.camera_callback,
                10)
            self.subscription
            self.bridge = CvBridge()


    def camera_callback(self, data):
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            filename = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_def>
            cascade = cv2.CascadeClassifier(filename)
            face = cascade.detectMultiScale(cv_image)

            if len(face) > 0:
                for r in face:
                    x, y = r[0:2]
                    width, height = r[0:2] + r[2:4]
                    cv2.rectangle(cv_image, (x, y), (width, height), (255, 255, 255>
            else:
                self.get_logger().info('not detect face')

            ros_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.publisher_.publish(ros_image)


    def main(args=None):
        rclpy.init(args=args)

        image_publisher = ImagePublisher()

        rclpy.spin(image_publisher)

        image_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ nano src/cv_test/setup.py

編集。

.. code-block:: python
    :emphasize-lines: 31
    :caption: setup.py

    from setuptools import find_packages, setup

    package_name = 'cv_test'

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
        maintainer='ubuntu',
        maintainer_email='ubuntu@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'img_publisher = cv_test.image_view:main',
                'gray_publisher = cv_test.gray:main',
                'circle_publisher = cv_test.circle:main',
                'binary_publisher = cv_test.binary:main',
                'edge_publisher = cv_test.edge:main',
                'face_publisher = cv_test.face_detect:main',
                'eye_publisher = cv_test.eye_detect:main',
                'cam_publisher = cv_test.cam_gray:main',
                'cam_face_publisher = cv_test.cam_face_detect:main',
            ],
        },
    )

ビルド。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ colcon build --packages-select cv_test
    Starting >>> cv_test 
    Finished <<< cv_test [2.14s]          

    Summary: 1 package finished [2.37s]

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ source install/local_setup.bash

cv_testパッケージのcam_face_publisherノードの実行

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ ros2 run cv_test cam_face_publisher

v4l2_camera_nodeの実行。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ ros2 run v4l2_camera v4l2_camera_node

rqt_image_viewで画像を表示。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ ros2 run rqt_image_view rqt_image_view

|

演習2「cam_face_detect.pyを実行するlaunchファイルを作ってください」
===================================================================

ファイル名はcam_face_detect_launch_pyとします。

|

launchディレクトリを作成。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ mkdir src/cv_test/launch

cam_face_detect_launch.pyを作成。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ nano src/cv_test/launch/cam_face_detect_launch.py

編集。

.. code-block:: console
    :caption: cam_face_detect_launch.py

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                name="camera",
                package="v4l2_camera",
                executable="v4l2_camera_node",
            ),
            Node(
                name="view",
                package="rqt_image_view",
                executable="rqt_image_view",
            ),
            Node(
                name="cam_face",
                package="cv_test",
                executable="cam_face_publisher",
            ),
        ])

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ nano src/cv_test/setup.py 

編集。

.. code-block:: console
    :emphasize-lines: 1-2, 16
    :caption: setup.py

    import os
    from glob import glob

    from setuptools import find_packages, setup

    package_name = 'cv_test'

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
                'img_publisher = cv_test.image_view:main',
                'gray_publisher = cv_test.gray:main',
                'circle_publisher = cv_test.circle:main',
                'binary_publisher = cv_test.binary:main',
                'edge_publisher = cv_test.edge:main',
                'face_publisher = cv_test.face_detect:main',
                'eye_publisher = cv_test.eye_detect:main',
                'cam_publisher = cv_test.cam_gray:main',
                'cam_face_publisher = cv_test.cam_face_detect:main',
            ],
        },
    )

ビルド。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ colcon build --packages-select cv_test
    Starting >>> cv_test 
    Finished <<< cv_test [2.03s]          

    Summary: 1 package finished [2.25s]

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ source install/setup.bash

launchファイルの実行。

.. code-block:: console

    ubuntu@mbc112:~/ros2_ws$ ros2 launch cv_test cam_face_detect_launch.py

|

演習3「Raspberry Piに接続したWebカメラの画像をPCに表示してください」
====================================================================

パッケージのアップデート。

.. code-block:: console

    pi@zumo01:~$ sudo apt-get update

v4l2-cameraをインストール。

.. code-block:: console

    pi@zumo01:~$ sudo apt-get install ros-jazzy-v4l2-camera


カメラが認識されているか確認。/dev/video0が表示されていていればOK。

.. code-block:: console

    pi@zumo01:~$ ls /dev/video*
    /dev/video0   /dev/video23  /dev/video29  /dev/video35
    /dev/video1   /dev/video24  /dev/video30  /dev/video36
    /dev/video19  /dev/video25  /dev/video31  /dev/video37
    /dev/video20  /dev/video26  /dev/video32
    /dev/video21  /dev/video27  /dev/video33
    /dev/video22  /dev/video28  /dev/video34

v4l2_camera_nodeの実行。

.. code-block:: console

    pi@zumo01:~$ ros2 run v4l2_camera v4l2_camera_node

トピックの確認。

.. code-block:: console

    ubuntu@mbc112:~$ ros2 topic list
    /camera_info
    /image_raw
    /parameter_events
    /rosout

rqt_image_viewで画像を表示。

.. code-block:: console

    ubuntu@mbc112:~$ ros2 run rqt_image_view rqt_image_view

.. note::

    rqt_image_viewが正常に動作しない？

.. note::

    PCからRaspberry Piのカメラを起動できる？
