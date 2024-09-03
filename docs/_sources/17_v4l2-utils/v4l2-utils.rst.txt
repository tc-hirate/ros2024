============================================================
v4l2-utils
============================================================

パッケージのアップデート。

.. code-block:: console

    pi@zumo05:~$ sudo apt update

ユーティリティのインストール。

.. code-block:: console

    pi@zumo05:~$ sudo apt install v4l-utils

カメラが認識されているか確認。

.. code-block:: console

    pi@zumo05:~$ v4l2-ctl --list-devices
    pispbe (platform:1000880000.pisp_be):
            /dev/video20
            /dev/video21
            /dev/video22
            /dev/video23
            /dev/video24
            /dev/video25
            /dev/video26
            /dev/video27
            /dev/video28
            /dev/video29
            /dev/video30
            /dev/video31
            /dev/video32
            /dev/video33
            /dev/video34
            /dev/video35
            /dev/video36
            /dev/video37
            /dev/media0
            /dev/media1

    rpivid (platform:rpivid):
            /dev/video19
            /dev/media2

    UVC Camera (046d:0825) (usb-xhci-hcd.1-1):
            /dev/video0
            /dev/video1
            /dev/media3

カメラのフォーマットを確認。

.. code-block:: console

    pi@zumo05:~$ v4l2-ctl --list-formats-ext
    ioctl: VIDIOC_ENUM_FMT
            Type: Video Capture

            [0]: 'YUYV' (YUYV 4:2:2)
                    Size: Discrete 640x480
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 160x120
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 176x144
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 320x176
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 320x240
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 352x288
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 432x240
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 544x288
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 640x360
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 752x416
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 800x448
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 800x600
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 864x480
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 960x544
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 960x720
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 1024x576
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 1184x656
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 1280x720
                            Interval: Discrete 0.133s (7.500 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 1280x960
                            Interval: Discrete 0.133s (7.500 fps)
                            Interval: Discrete 0.200s (5.000 fps)
            [1]: 'MJPG' (Motion-JPEG, compressed)
                    Size: Discrete 640x480
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 160x120
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 176x144
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 320x176
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 320x240
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 352x288
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 432x240
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 544x288
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 640x360
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 752x416
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 800x448
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 800x600
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 864x480
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 960x544
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 960x720
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 1024x576
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 1184x656
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 1280x720
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)
                    Size: Discrete 1280x960
                            Interval: Discrete 0.033s (30.000 fps)
                            Interval: Discrete 0.040s (25.000 fps)
                            Interval: Discrete 0.050s (20.000 fps)
                            Interval: Discrete 0.067s (15.000 fps)
                            Interval: Discrete 0.100s (10.000 fps)
                            Interval: Discrete 0.200s (5.000 fps)

カメラのフレームレートを確認。

.. code-block:: console

    pi@zumo05:~$ v4l2-ctl -P
    Streaming Parameters Video Capture:
            Capabilities     : timeperframe
            Frames per second: 30.000 (30/1)
            Read buffers     : 0

カメラのフレームレートを変更。

.. code-block:: console

    pi@zumo05:~$ v4l2-ctl -p 5
    Frame rate set to 5.000 fps

変更したカメラのフレームレートを確認。

.. code-block:: console

    pi@zumo05:~$ v4l2-ctl -P
    Streaming Parameters Video Capture:
            Capabilities     : timeperframe
            Frames per second: 5.000 (5/1)
            Read buffers     : 0
