[English](README.md) | [日本語](README_jp.md) 

# 箱庭 ROS シミュレータ

[TOPPERSプロジェクト箱庭WG](https://toppers.github.io/hakoniwa)では，IoT／クラウドロボティクス時代の仮想シミュレーション環境である『箱庭』の研究開発を進めています．

本リポジトリでは，箱庭上で ROS 2 プログラムのシミュレーションを簡単にお試しできる環境を公開しています．


## 想定する PC 環境

* Windows 環境: Windows 10/11
  * Ubuntu 20.04 LTS on WSL2/WSLg
* Linux 環境: Ubuntu 20.04 LTS
* Mac 環境: macOS Catalina ver.10.15.7

Windows 環境では，操作は全てWSL2/Linuxのシェル上で行います．WSL2のファイルシステム配下（`/home/${USER}/`以下）ではなくWindowsファイルシステム配下（`/mnt/c/`以下）で実行してください．

## PC環境の準備

### 本リポジトリのclone

現在の最新版は **v1.1.4** です．
「[バージョン情報・更新履歴](/appendix/version.md)」も参照してください（バージョン番号は[Git/GitHubのtag/release](https://github.com/toppers/hakoniwa-ros2sim/releases)および[Docker Hubのtag番号](https://hub.docker.com/r/toppersjp/hakoniwa-ros2sim/tags)に対応しています）

ターミナルで下記を実行して本リポジトリをcloneしてください．

```
git clone --recursive -b v1.1.4 https://github.com/toppers/hakoniwa-ros2sim.git
```

### Docker 環境

本シミュレータでは Docker を利用します．

実験的にネイティブのLinux環境（WSL2含む）での動作を試行しています．ネイティブ環境での動作手順は [appendix/native.md](/appendix/native.md) をご参照ください．

#### Mac 環境の場合

[Docker Desktop for Mac](https://docs.docker.com/desktop/mac/install/) の利用を推奨します．

#### Windows/WSL2 または Linux 環境の場合

Docker Engineがインストールされている必要があります．WSL2またはLinuxのターミナルで下記のコマンドの結果が同じように出力されていれば，すでにインストール済みです（`$`から始まる行は実行するコマンドを示しています）．

```
$ which docker
/usr/bin/docker
$ service --status-all |& grep docker
 [ + ]  docker   # または " [ - ]  docker "
$ service docker status
 * Docker is running   # または " * Docker is not running "
```

Docker Engineのインストールはやや手数が多いため，下記の公式マニュアルにある実行コマンドを["toppersjp/hakoniwa-single_robot リポジトリの `docker/install-docker.bash`](https://github.com/toppers/hakoniwa-single_robot/blob/main/docker/install-docker.bash) にまとめています．本スクリプトの実行時に問題がありましたら，公式マニュアルの手順に従ってインストールを進めてください．  
[Install Docker Engine on Ubuntu | Docker Documentation](https://docs.docker.com/engine/install/ubuntu/)

スクリプトを用いたDockerのインストールには，下記のように実行してください．

```
wget https://raw.githubusercontent.com/toppers/hakoniwa-single_robot/main/docker/install-docker.bash
bash install-docker.bash
```

`service docker status` の結果が " * Docker is not running " の場合は，Dockerを起動してください．

```
sudo service docker start
```

次のように出力されていれば，Dockerが起動しています．

```
 * Starting Docker: docker                           [ OK ] 
```

また，ユーザが `docker` のグループに所属していることを想定しています．そうでない場合は，次のコマンドを実行してください．

```
sudo gpasswd -a $USER docker
sudo chgrp docker /var/run/docker.sock
sudo service docker restart
```

上記のコマンド実行結果は，ターミナルに再ログインしてから有効となります．

### Unity 環境のインストール

* Unity Hub 3.1.1 以降
* Unity Editor 2021.3.0f1
  * Unity Hub の「Installs > Install Editor」画面に本バージョンが表示されない場合は，[Unity Dowonload Archive](https://unity3d.com/get-unity/download/archive) の本バージョンの "Unity Hub" をクリックしてインストールできます． 

## シミュレータの導入手順

### Dockerイメージの展開

シミュレータの実行環境は，ビルド済みのDocker imageをDocker Hubにて公開しています．

https://hub.docker.com/r/toppersjp/hakoniwa-ros2sim

次のコマンドを実行してください．Dockerイメージののpullと展開を行います．

```
bash docker/pull-image.bash
```

\[補足：開発者向け情報\] Dockerイメージの作成用に `docker/create-image.bash` があります．

### dockerを起動する

ターミナルを2個起動します（以降の説明では，ターミナルAおよびターミナルBと呼びます）．

ターミナルAでdockerコンテナを起動します．

```
bash docker/run.bash
```

Mac環境の場合は，ネットワークポート名（例："en0"）を引数に指定する必要があります．
ポート名は `ifconfig` コマンド等で確認できます．

```
bash docker/run.bash <port>
```

### 起動した dockerコンテナ上で箱庭のROS環境をインストール

```
bash hako-install.bash opt all
```

### Unity プロジェクトを開く

Unity Hubを起動し，右上の「開く」をクリックして、先ほどクローンしたROS対応版箱庭ソース上の以下のディレクトリを指定します．

場所：ros2/unity/tb3

![](https://camo.qiitausercontent.com/34b3dee89c42787380e888af0bb4a321c866ebe3/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f39353336626137632d383932372d383037302d613934372d3361643839396436363731622e706e67)

起動すると，以下の画面が表示されます．

![](https://camo.qiitausercontent.com/118f0af1ef4fdf3cdb4c2e7017d7ac92e2079943/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f33303965666239392d643663612d383937612d356233342d3334386233333763353339322e706e67)


この状態で、画面左下にある「プロジェクトビュー」の「Assets/Scenes」を選択すると、画面下に「Toppers_Course」というシーンがありますので，これをダブルクリックしましょう．

![](https://camo.qiitausercontent.com/34335041b2e8814a02a877d7d9de420ed0368d29/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f30303730336135642d373031342d656331322d396334612d3161616230366230323864612e706e67)


以下のようにコースが表示されます．

![](https://camo.qiitausercontent.com/5c2d040c568dadccec8e3347349d949716f5ce11/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f64666365393531362d663036372d303364652d366131622d3432376462653263396232332e706e67)


## シミュレータの実行手順

次の対象を例題として実行手順を説明します．

* ROS 側の制御プログラム：`src/tb3/src/tb3ctrl.cpp`
* Unity側のロボット：`TB3RoboModel`


### 準備

ターミナルAとBの両方ででdockerコンテナに入ります．

ターミナルAで Dockerコンテナを終了させていた場合は，改めて起動してください，

```
bash docker/run.bash
```

ターミナルB側は，以下のコマンドで入ります．

```
bash docker/attach.bash
```

### ターミナルAでの操作

ターミナルAでROS-TCP-ENDPOINTを起動しましょう．

```
# bash launch.bash
```

### ターミナルBでの操作

ターミナルBでROS2プログラムを起動しましょう．

```
# bash run.bash tb3 TB3RoboModel
```

### Unityのシミュレーション開始する
Unityのシミュレーション開始ボタンをクリックすると，以下の起動画面が出てきます．

![](https://camo.qiitausercontent.com/8aa80400f8a6b9527febde6edc5778187dc5f1cd/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f64396265303530612d393662352d353032322d653230642d3964616332633166613430322e706e67)


この状態で，Unity側の「開始」ボタンをクリックすると箱庭のシミュレーション開始し，TurtleBot3が動き出します．

### 動作例

![動作例](https://camo.qiitausercontent.com/6aae22e5ac3d57f9faaf43c75d9eee84eb0d0dc8/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f32316433333837622d336663652d303430322d623734362d3666353333353763643239612e676966)

この動画の各ウィンドウは，それぞれ次の通り対応しています．

- 右上：ターミナルA
- 右下：ターミナルB
- 左：Unity


## Contributing

本リポジトリで公開している「箱庭 ROS シミュレータ」について，ご意見や改善の提案などをぜひ [こちらのGitHub Discussions](https://github.com/toppers/hakoniwa/discussions/categories/idea-request) でお知らせください．改修提案の [Pull Requests](https://github.com/toppers/hakoniwa-ros2sim/pulls) も歓迎いたします．

## TODO

- [ ] SLAMやNav2の動作例を示す ([#19](https://github.com/toppers/hakoniwa-ros2sim/issues/19))


## 謝辞
* TurtleBot3 の Unity パッケージの設計と作成にあたっては，宝塚大学 東京メディア芸術学部 吉岡章夫准教授および学部生の杉崎涼志さん，木村明美さんにご協力いただきました．
* TurtleBot3 のUnity アセットは，株式会社ロボティズ様より提供いただいたデータを基に作成しています．ご協力いただき深く感謝いたします．

## ライセンス

[TOPPERSライセンス](https://www.toppers.jp/license.html)で公開しています．  
著作権者はTOPPERSプロジェクト箱庭ワーキンググループです．詳細は[LICENSE.md](./LICENSE.md)をご参照ください．
