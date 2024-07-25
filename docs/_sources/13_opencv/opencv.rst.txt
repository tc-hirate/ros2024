============================================================
OpenCVの使い方
============================================================

ROS2でOpenCVライブラリを使う方法を説明します。

詳しくは、 `OpenCV-Python Tutorials <https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html>`_ などを参照。

|

OpenCVのインストール
============================================================

OpenCVをインストールします。

.. code-block:: console

    ubuntu@mbc084:~$ sudo apt install python3-opencv

|

パッケージの作成
============================================================

cv_testという名前のパッケージを作ります。

.. code-block:: console

    ubuntu@mbc084:~$ cd ~/ros2_ws/src/
    ubuntu@mbc084:~/ros2_ws/src$ ros2 pkg create --build-type ament_python cv_test

|

画像の準備
============================================================

画像データをGitHubからダウンロードします。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws/src$ cd
    ubuntu@mbc084:~$ sudo apt install git
    ubuntu@mbc084:~$ git clone https://github.com/tc-hirate/zumo_ros.git

ダウンロードした画像データ（/zumo_ros/imgの下にあります）を/ros_ws/src/cv_test/cv_testディレクトリに移動してください。

.. code-block:: console

    ubuntu@mbc084:~$ cp zumo_ros/img/*.png ros2_ws/src/cv_test/cv_test/
    ubuntu@mbc084:~$ ls ros2_ws/src/cv_test/cv_test/
    __init__.py  gradient.png  img2.png  sagaairport.png
    base.png     img1.png      lena.png

|

画像の表示
============================================================

OpenCVライブラリを使って画像を読み込み、表示する方法について説明します。

ファイル名はimage_view.pyとします。

画像処理は次の手順で行っています。

- 画像を取得する（cv_image）
- ROSのImage型に変換する（ros_image）

このプログラムでは、cv_bridgeというライブラリをインポートしています。

cv_bridgeは、OpenCVの標準データ形式であるCV::MatをROSのメッセージであるsensor_msgs/Imageに変換します。

image_view.pyの作成

.. code-block:: console

    ubuntu@mbc084:~$ cd ros2_ws/
    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/cv_test/image_view.py

|

編集

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'image_data', 10)
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.bridge = CvBridge()

        def timer_callback(self):
            cv_image = cv2.imread('./src/cv_test/cv_test/lena.png')
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

|

package.xmlを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/package.xml

編集。

.. code-block:: none
    :emphasize-lines: 10-13

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematyp>
    <package format="3">
      <name>cv_test</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>
      <license>TODO: License declaration</license>

      <exec_depend>rclpy</exec_depend>
      <exec_depend>sensor_msgs</exec_depend>
      <exec_depend>cv_bridge</exec_depend>
      <exec_depend>opencv2</exec_depend>

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

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 23

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
            ],
        },
    )

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select cv_test

|

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/local_setup.bash

|

cv_testパッケージのimg_publisherノードの実行

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run cv_test img_publisher

|

トピックで/image_dataが出力されているか確認しましょう。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 topic list
    /image_data
    /parameter_events
    /rosout

画像を見るためにrqt_image_viewというツールを使います。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run rqt_image_view  rqt_image_view

|

.. image:: ./img/opencv_img_01.png
   :align: center

|

/image_dataを選択すると、画像が表示されます。

.. image:: ./img/opencv_img_02.png
   :align: center

|

画像をグレースケールに変換する
============================================================

OpenCVライブラリを使ってカラー画像を グレースケール画像に変換します。

image_view.pyをコピーしてgray.pyを作ってください。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ cp src/cv_test/cv_test/image_view.py src/cv_test/cv_test/gray.py

|

gray.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/cv_test/gray.py

|

編集。

.. code-block:: python
    :emphasize-lines: 18, 19

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'image_data', 10)
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.bridge = CvBridge()

        def timer_callback(self):
            cv_image = cv2.imread('./src/cv_test/cv_test/lena.png')
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

|

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 24

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
            ],
        },
    )

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select cv_test

|

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/local_setup.bash

|

cv_testパッケージのgray_publisherノードの実行

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run cv_test gray_publisher

|

確認。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run rqt_image_view  rqt_image_view

|

.. image:: ./img/opencv_img_03.png
   :align: center

|

円を描く
============================================================

OpenCVライブラリを使って図形を描きます。

image_view.pyをコピーしてcircle.pyを作ってください。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ cp src/cv_test/cv_test/image_view.py src/cv_test/cv_test/circle.py

|

gray.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/cv_test/circle.py

|

編集

.. code-block:: python
    :emphasize-lines: 17-19

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'image_data', 10)
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.bridge = CvBridge()

        def timer_callback(self):
            cv_image = cv2.imread('./src/cv_test/cv_test/base.png')
            cv_circle_image = cv2.circle(cv_image, (250, 250), 100, (0, 255, 0), 3)
            ros_image = self.bridge.cv2_to_imgmsg(cv_circle_image, 'bgr8')
            self.publisher_.publish(ros_image)


    def main(args=None):
        rclpy.init(args=args)

        image_publisher = ImagePublisher()

        rclpy.spin(image_publisher)

        image_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

|

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 25

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
            ],
        },
    )

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select cv_test

|

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/local_setup.bash

|

cv_testパッケージのcircle_publisherノードの実行

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run cv_test circle_publisher

|

確認。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run rqt_image_view  rqt_image_view

|

.. image:: ./img/opencv_img_04.png
   :align: center

|

500ピクセル×500ピクセルの黒色の画像に円を描いています。

円は、中心座標が(250,250)、半径が100、色が緑、線の太さが3です。

OpenCVライブラリを使って線分（cv2.line()）、長方形（cv2.rectangle()）、文字列（cv2.putText）なども描いてみましょう。

.. code-block:: python

    cv_circle_image = cv2.line(cv_circle_image, (50, 50), (200, 100), (255, 0, 0), thickness=4)

.. code-block:: python

    cv_circle_image = cv2.rectangle(cv_circle_image, (300, 50), (450, 150), (0, 0, 255), thickness=4)

.. code-block:: python

    cv_circle_image = cv2.putText(cv_circle_image, 'Hirate', (100, 400), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), thickness=2)

|

画像を2値化する
============================================================

OpenCVライブラリを使って画像を2値化します。

image_view.pyをコピーしてbinary.pyを作ってください。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ cp src/cv_test/cv_test/image_view.py src/cv_test/cv_test/binary.py

|

binary.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/cv_test/binary.py

|

編集

.. code-block:: python
    :emphasize-lines: 17-20

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'image_data', 10)
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.bridge = CvBridge()

        def timer_callback(self):
            cv_image = cv2.imread('./src/cv_test/cv_test/gradient.png')
            cv_gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            ret, cv_binary_image = cv2.threshold(cv_gray_image, 127, 255, cv2.THRESH_BINARY)
            ros_image = self.bridge.cv2_to_imgmsg(cv_binary_image, 'mono8')
            self.publisher_.publish(ros_image)


    def main(args=None):
        rclpy.init(args=args)

        image_publisher = ImagePublisher()

        rclpy.spin(image_publisher)

        image_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

|

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 26

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
            ],
        },
    )

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select cv_test

|

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/local_setup.bash

|

cv_testパッケージのbinary_publisherノードの実行

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run cv_test binary_publisher

|

確認。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run rqt_image_view  rqt_image_view

|

.. image:: ./img/opencv_img_05.png
   :align: center

|

しきい値を変更するとどうなるか確認しましょう。

（例）80

.. code-block:: python

    ret, cv_binary_image = cv2.threshold(cv_gray_image, 80, 255, cv2.THRESH_BINARY)

（例）160

.. code-block:: python

    ret, cv_binary_image = cv2.threshold(cv_gray_image, 160, 255, cv2.THRESH_BINARY)

|

エッジ検出フィルタを使う
============================================================

OpenCVライブラリを使ってエッジの検出をします。

エッジ検出フィルタには、ラプラシアン、Sobel、Canny法などがあります。

ここでは、Canny法を使います。

image_view.pyをコピーしてedge.pyを作ってください。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ cp src/cv_test/cv_test/image_view.py src/cv_test/cv_test/edge.py

|

edge.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/cv_test/edge.py

|

編集

.. code-block:: python
    :emphasize-lines: 18-20

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'image_data', 10)
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.bridge = CvBridge()

        def timer_callback(self):
            cv_image = cv2.imread('./src/cv_test/cv_test/lena.png')
            cv_gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            cv_canny_image = cv2.Canny(cv_gray_image, threshold1=100, threshold2=200)
            ros_image = self.bridge.cv2_to_imgmsg(cv_canny_image, 'mono8')
            self.publisher_.publish(ros_image)


    def main(args=None):
        rclpy.init(args=args)

        image_publisher = ImagePublisher()

        rclpy.spin(image_publisher)

        image_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

|

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 27

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
            ],
        },
    )

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select cv_test

|

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/local_setup.bash

|

cv_testパッケージのedge_publisherノードの実行

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run cv_test edge_publisher

|

確認。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run rqt_image_view  rqt_image_view

|

.. image:: ./img/opencv_img_06.png
   :align: center

|

threshold1とthreshold2の値を変えるとどうなるか試してください。

（例）threshold1=50, threshold2=100

.. code-block:: python

    cv_canny_image = cv2.Canny(cv_gray_image, threshold1=50, threshold2=100)

|

演習3「OpenCVライブラリを使った顔検出」
============================================================

この演習では、img1.pngとimg2.pngを使います。

|

（１）img1.pngを読み込んで、顔を検出するプログラムを作ってください。
--------------------------------------------------------------------

ファイル名はface_detect.pyとします。

OpenCVでオブジェクトを検出するためには、CascadeClassifier関数で特徴量を学習したXMLファイルを読み込みます。

そして、取得したCascadeClassifierオブジェクトのdetectMultiScale関数を呼び出します。

顔を検出する場合は、haarcascade_frontalface_default.xmlを使います。

|

face_detect.pyの作成。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/cv_test/face_detect.py

|

編集

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'image_data', 10)
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.bridge = CvBridge()

        def timer_callback(self):
            cv_image = cv2.imread('./src/cv_test/cv_test/img1.png')

            filename = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
            cascade = cv2.CascadeClassifier(filename)
            face = cascade.detectMultiScale(cv_image)

            if len(face) > 0:
                for r in face:
                    x, y = r[0:2]
                    width, height = r[0:2] + r[2:4]
                    cv2.rectangle(cv_image, (x, y), (width, height), (255, 255, 255), thickness=2)
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

|

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 28

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
            ],
        },
    )

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select cv_test

|

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/local_setup.bash

|

cv_testパッケージのface_publisherノードの実行

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run cv_test face_publisher

|

確認。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run rqt_image_view  rqt_image_view

|

少しずれてしまいました。

.. image:: ./img/opencv_img_07.png
   :align: center

|

（２）face_detect.pyで読み込む画像を img2.pngに変更して、顔を検出してください。
-------------------------------------------------------------------------------

正しく検出できました。

.. image:: ./img/opencv_img_08.png
   :align: center

|

（３）img1.pngを読み込んで、目を検出するプログラムを作ってください。
--------------------------------------------------------------------

ファイル名はeye_detect.pyとします。

XMLファイルはhaarcascade_eye.xmlを使います。

|

face_detect.pyをコピーしてeye_detect.pyを作ってください。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ cp src/cv_test/cv_test/face_detect.py src/cv_test/cv_test/eye_detect.py

|

eye_detect.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/cv_test/eye_detect.py

|

編集

.. code-block:: python
    :emphasize-lines: 17, 19, 29

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class ImagePublisher(Node):

        def __init__(self):
            super().__init__('image_publisher')
            self.publisher_ = self.create_publisher(Image, 'image_data', 10)
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.bridge = CvBridge()

        def timer_callback(self):
            cv_image = cv2.imread('./src/cv_test/cv_test/img1.png')

            filename = '/usr/share/opencv4/haarcascades/haarcascade_eye.xml'
            cascade = cv2.CascadeClassifier(filename)
            face = cascade.detectMultiScale(cv_image)

            if len(face) > 0:
                for r in face:
                    x, y = r[0:2]
                    width, height = r[0:2] + r[2:4]
                    cv2.rectangle(cv_image, (x, y), (width, height), (255, 255, 255), thickness=2)
            else:
                self.get_logger().info('not detect eye')

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

|

setup.pyを開く。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ nano src/cv_test/setup.py

|

編集。

.. code-block:: python
    :emphasize-lines: 29

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
            ],
        },
    )

|

ビルド。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ colcon build --packages-select cv_test

|

セットアップファイルの反映。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ source install/local_setup.bash

|

cv_testパッケージのface_publisherノードの実行

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run cv_test eye_publisher

|

確認。

.. code-block:: console

    ubuntu@mbc084:~/ros2_ws$ ros2 run rqt_image_view  rqt_image_view

|

正しく検出できました。

.. image:: ./img/opencv_img_09.png
   :align: center

|


（４）eye_detect.pyで読み込む画像を img2.pngに変更して、顔を検出してください。
-------------------------------------------------------------------------------

口も検出してしまいました。

.. image:: ./img/opencv_img_10.png
   :align: center

|
