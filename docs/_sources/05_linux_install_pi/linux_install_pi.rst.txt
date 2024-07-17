============================================================
Linuxのインストール（Raspberry Pi）
============================================================

Linuxはオープンソースのオペレーティングシステム（OS）です。

Raspberry PiではUbuntu Server 24.04 LTSを使います。

|

Raspberry Pi Imagerのインストール
============================================================

公式のディスクイメージ書き込みツールRaspberry Pi Imagerを使ってSDカードにディスクイメージを書き込みます。

詳しくは、 `Raspberry Pi OSのHP <https://www.raspberrypi.com/software/>`_ を参照してください。

以下のコマンドを実行してRaspberry Pi Imagerをインストール。

.. code-block:: console

    ubuntu@mbc084:~$ sudo apt install rpi-imager

|

ディスクイメージの書き込み
============================================================

Raspberry Pi Imagerを起動。

[デバイスを選択]ボタンをクリック。

.. image:: ./images/linux_install_pi_img_01.png

|

「Raspberry Pi 5」を選択。

.. image:: ./images/linux_install_pi_img_02.png

|

[OSを選択]ボタンをクリック。

.. image:: ./images/linux_install_pi_img_03.png

|

「Other general-purpose OS」を選択。

.. image:: ./images/linux_install_pi_img_04.png

|

「Ubuntu」を選択。

.. image:: ./images/linux_install_pi_img_05.png

|

［Ubuntu Server 24.04.2 LTS (64bit)］を選択。

.. image:: ./images/linux_install_pi_img_06.png

|

SDカードを挿入して、[ストレージを選択]ボタンをクリック。

.. image:: ./images/linux_install_pi_img_07.png

|

挿入したSDカードを選択。

.. image:: ./images/linux_install_pi_img_08.png

|

[次へ]ボタンをクリック。

.. image:: ./images/linux_install_pi_img_09.png

|

[設定を編集する]ボタンをクリック。

.. image:: ./images/linux_install_pi_img_10.png

|

次の通り入力し[保存]ボタンをクリック。

- 「ホスト名」にチェック
- ホスト名：zumo01.local
- 「ユーザー名とパスワードを設定する」にチェック
- ユーザー名：pi
- パスワード：hirate2020
- 「ロケールを設定する」にチェック
- タイムゾーン：Asia/Tokyo
- キーボードレイアウト：jp

.. image:: ./images/linux_install_pi_img_11.png

|

[はい]ボタンをクリック。

.. image:: ./images/linux_install_pi_img_12.png

|

[はい]ボタンをクリック。

.. image:: ./images/linux_install_pi_img_13.png

|

書き込み中。

.. image:: ./images/linux_install_pi_img_14.png

|

［続ける］ボタンをクリック。

.. image:: ./images/linux_install_pi_img_15.png

|

Raspberry Pi Imagerを終了し、SDカードをUnmountして取り出す。

|

Linuxへのログイン
============================================================

Raspberry Piにモニター、キーボード、LANケーブルを接続してください。

SDカードを挿入して電源を入れます。

しばらくすると次のような場面が表示されます。

.. code-block:: console

    Ubuntu 24.04 LTS zumo01 tty1

    zumo01 login: pi
    Password: hirate2020

ユーザー名にpi、パスワードにhirate2020を入力してログイン。

※パスワードは表示されません

.. code-block:: console

    Welcome to Ubuntu 24.04 LTS (GNU/Linux 6.8.0-1004-raspi aarch64)
    ・・・
    pi@zumo01:~$

|

ssh接続
============================================================

PCからRaspberry Piを遠隔操作するためにssh接続をします。

今回は、Ubuntuに標準でインストールされているRemminaを使います。

はじめに、Raspberry PiのIPアドレスを調べます。

.. code-block:: console
    :emphasize-lines: 10

    pi@zumo00:~$ ip a
    1: lo:  mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
        link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
        inet 127.0.0.1/8 scope host lo
           valid_lft forever preferred_lft forever
        inet6 ::1/128 scope host 
           valid_lft forever preferred_lft forever
    2: eth0:  mtu 1500 qdisc fq_codel state UP group default qlen 1000
        link/ether b8:27:eb:71:91:bf brd ff:ff:ff:ff:ff:ff
        inet 192.168.1.31/24 brd 192.168.1.255 scope global dynamic eth0
           valid_lft 258007sec preferred_lft 258007sec
        inet6 fe80::ba27:ebff:fe71:91bf/64 scope link 
           valid_lft forever preferred_lft forever
    3: wlan0:  mtu 1500 qdisc noop state DOWN group default qlen 1000
        link/ether b8:27:eb:24:c4:ea brd ff:ff:ff:ff:ff:ff

Remminaを起動します。

次のウィンドウが出てきたら、［閉じる］ボタンをクリック。

.. image:: ./img/ssh_img_01.png
   :align: center

|

［New connection profile］ボタンをクリック。

.. image:: ./img/ssh_img_02.png
   :align: center

|

「リモートデスクトップの設定」ウィンドウが立ち上がります。

必要な情報を入力して、［保存］ボタンをクリック。

.. image:: ./img/ssh_img_03.png
   :align: center

|

zumo00という名前の接続が追加されました。

zumo00をダブルクリックするとラズベリーパイと接続されます。

.. image:: ./img/ssh_img_04.png
   :align: center

|

zumo00と接続されました。

以降、Raspberry Piの操作はこの画面から行います。

.. image:: ./img/ssh_img_05.png
   :align: center


|

無線LANの設定
============================================================

現在のネットワークの設定を確認します。

.. code-block:: console

    pi@zumo00:~$ cat /etc/netplan/50-cloud-init.yaml
    # This file is generated from information provided by the datasource.  Changes
    # to it will not persist across an instance reboot.  To disable cloud-inits
    # network configuration capabilities, write a file
    # /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
    # network: {config: disabled}
    network:
        ethernets:
            eth0:
                dhcp4: true
                optional: true
        version: 2

現在の設定ファイルをコピーして、新しい設定ファイルを作ります。

.. code-block:: console

    pi@zumo00:~$ sudo cp /etc/netplan/50-cloud-init.yaml /etc/netplan/99_config.yaml
    [sudo] password for pi:

新しい設定ファイルを編集します。

.. code-block:: console

    $ sudo nano /etc/netplan/99_config.yaml

IPアドレスには、自分が使っているZumoに割り振られたIPアドレスを入力してください。

.. code-block:: console
    :emphasize-lines: 11 - 24

    # This file is generated from information provided by the datasource.  Changes
    # to it will not persist across an instance reboot.  To disable cloud-inits
    # network configuration capabilities, write a file
    # /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
    # network: {config: disabled}
    network:
        ethernets:
            eth0:
                dhcp4: true
                optional: true
        wifis:
            wlan0:
                dhcp4: false
                optional: true
                addresses: [192.168.1.250/24]
                routes:
                  - to: default
                    via: 192.168.1.1
                nameservers:
                    addresses: [192.168.1.1]
                    search: []
                access-points:
                    htc-s-ap:
                        password: "E4LEHeJnS7"
        version: 2

設定を反映させます。

.. code-block:: console

    $ sudo netplan apply

Raspberry Piを終了します。（Remminaのssh接続は切断されます）

.. code-block:: console

    $ sudo shutdown -h now

wifiでネットワークに接続されているか確認します。
LANケーブルを抜いてからRaspberry Piの電源を入れてください。

IPアドレスを確認します。

.. code-block:: console
    :emphasize-lines: 12

    pi@zumo00:~$ ip a
    1: lo:  mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
        link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
        inet 127.0.0.1/8 scope host lo
           valid_lft forever preferred_lft forever
        inet6 ::1/128 scope host 
           valid_lft forever preferred_lft forever
    2: eth0:  mtu 1500 qdisc fq_codel state DOWN group default qlen 1000
        link/ether b8:27:eb:71:91:bf brd ff:ff:ff:ff:ff:ff
    3: wlan0:  mtu 1500 qdisc fq_codel state UP group default qlen 1000
        link/ether b8:27:eb:24:c4:ea brd ff:ff:ff:ff:ff:ff
        inet 192.168.1.250/24 brd 192.168.1.255 scope global wlan0
           valid_lft forever preferred_lft forever
        inet6 fe80::ba27:ebff:fe24:c4ea/64 scope link 
           valid_lft forever preferred_lft forever

次のコマンドを実行して、インターネットとつながっているか確認してください。

.. code-block:: console

    pi@zumo00:~$ ping youtube.com
    PING youtube.com (172.217.27.78) 56(84) bytes of data.
    64 bytes from nrt12s15-in-f14.1e100.net (172.217.27.78): icmp_seq=1 ttl=113 time=61.6 ms
    64 bytes from nrt12s15-in-f14.1e100.net (172.217.27.78): icmp_seq=2 ttl=113 time=20.7 ms
    64 bytes from nrt12s15-in-f14.1e100.net (172.217.27.78): icmp_seq=3 ttl=113 time=19.2 ms
    ・・・

Remminaでwifi用の接続も作成しておきましょう。
名前はzumo00-wifiとします。

.. image:: ./img/ssh_img_06.png
   :align: center

|

.. image:: ./img/ssh_img_07.png
   :align: center
