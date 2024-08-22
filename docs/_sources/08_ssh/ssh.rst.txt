============================================================
SSH接続
============================================================

PCからRaspberry Piを遠隔操作するためにssh接続をします。

Remminaのインストール
============================================================

`RemminaのHP <https://remmina.org/how-to-install-remmina/>`_ の手順に従ってインストールします。

.. code-block:: console

    ubuntu@mbc112:~$ sudo apt-add-repository ppa:remmina-ppa-team/remmina-next
    ubuntu@mbc112:~$ sudo apt update
    ubuntu@mbc112:~$ sudo apt install remmina remmina-plugin-rdp remmina-plugin-secret

ssh接続
============================================================

はじめに、Raspberry PiのIPアドレスを調べます。

.. code-block:: console
    :emphasize-lines: 10

    pi@zumo01:~$ ip a
    1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
        link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
        inet 127.0.0.1/8 scope host lo
           valid_lft forever preferred_lft forever
        inet6 ::1/128 scope host noprefixroute 
           valid_lft forever preferred_lft forever
    2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc pfifo_fast state UP group default qlen 1000
        link/ether 2c:cf:67:28:d6:a0 brd ff:ff:ff:ff:ff:ff
        inet 192.168.1.45/24 metric 100 brd 192.168.1.255 scope global dynamic eth0
           valid_lft 258392sec preferred_lft 258392sec
        inet6 fe80::2ecf:67ff:fe28:d6a0/64 scope link 
           valid_lft forever preferred_lft forever
    3: wlan0: <BROADCAST,MULTICAST> mtu 1500 qdisc noop state DOWN group default qlen 1000
        link/ether 2c:cf:67:28:d6:a1 brd ff:ff:ff:ff:ff:ff

Remminaを起動し、[新しい接続プロファイル]ボタンをクリック。

.. image:: ./images/ssh_img_01.png

必要な情報を入力して、［保存］ボタンをクリック。

.. image:: ./images/ssh_img_02.png

「zumo01」が作成された。

.. image:: ./images/ssh_img_03.png

「zumo01」をダブルクリックして（右クリックから「接続」でも可）、ラズベリーパイと接続。

以降、Raspberry Piの操作はこの画面から行います。

.. image:: ./images/ssh_img_04.png

無線LANの設定
============================================================

現在のネットワークの設定を確認。

.. code-block:: console

    pi@zumo01:~$ sudo cat /etc/netplan/50-cloud-init.yaml 
    # This file is generated from information provided by the datasource.  Changes
    # to it will not persist across an instance reboot.  To disable cloud-init's
    # network configuration capabilities, write a file
    # /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
    # network: {config: disabled}
    network:
        ethernets:
            eth0:
                dhcp4: true
                optional: true
        version: 2

現在の設定ファイルをコピーして、新しい設定ファイルを作成。

.. code-block:: console

    pi@zumo01:~$ sudo cp /etc/netplan/50-cloud-init.yaml /etc/netplan/99_config.yaml

新しい設定ファイルを編集。

.. code-block:: console

    pi@zumo01:~$ sudo nano /etc/netplan/99_config.yaml

IPアドレスには、自分が使っているZumoに割り振られたIPアドレスを入力してください。

.. code-block:: console
    :emphasize-lines: 11 - 24
    :caption: 99_config.yaml

    # This file is generated from information provided by the datas>
    # to it will not persist across an instance reboot.  To disable>
    # network configuration capabilities, write a file
    # /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the>
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
                addresses: [192.168.1.241/24]
                routes:
                  - to: default
                    via: 192.168.1.1
                nameservers:
                    addresses: [192.168.1.1]
                    search: []
                access-points:
                    htc-s-ap:
                        password: "BHnAJtVbS7"
        version: 2

設定を反映。

.. code-block:: console

    pi@zumo01:~$ sudo netplan apply

Raspberry Piを終了。

.. code-block:: console

    pi@zumo01:~$ sudo poweroff

wifiでネットワークに接続されているか確認します。
LANケーブルを抜いてからRaspberry Piの電源を入れてください。

IPアドレスを確認。

.. code-block:: console
    :emphasize-lines: 12

    pi@zumo01:~$ ip a
    1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
        link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
        inet 127.0.0.1/8 scope host lo
           valid_lft forever preferred_lft forever
        inet6 ::1/128 scope host noprefixroute 
           valid_lft forever preferred_lft forever
    2: eth0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc pfifo_fast state DOWN group default qlen 1000
        link/ether 2c:cf:67:28:d6:a0 brd ff:ff:ff:ff:ff:ff
    3: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc pfifo_fast state UP group default qlen 1000
        link/ether 2c:cf:67:28:d6:a1 brd ff:ff:ff:ff:ff:ff
        inet 192.168.1.241/24 brd 192.168.1.255 scope global wlan0
           valid_lft forever preferred_lft forever
        inet6 fe80::2ecf:67ff:fe28:d6a1/64 scope link 
           valid_lft forever preferred_lft forever

次のコマンドを実行して、インターネットとつながっているか確認。

.. code-block:: console

    pi@zumo01:~$ ping youtube.com
    PING youtube.com (142.250.207.46) 56(84) bytes of data.
    64 bytes from nrt13s55-in-f14.1e100.net (142.250.207.46): icmp_seq=1 ttl=57 time=14.9 ms
    64 bytes from nrt13s55-in-f14.1e100.net (142.250.207.46): icmp_seq=2 ttl=57 time=18.0 ms
    64 bytes from nrt13s55-in-f14.1e100.net (142.250.207.46): icmp_seq=3 ttl=57 time=29.1 ms
    ・・・

Remminaでwifi用の接続も作成。
名前はzumo01-wifiとする。

.. image:: ./images/ssh_img_05.png
