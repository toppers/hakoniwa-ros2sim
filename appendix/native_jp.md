[English](native.md) | [日本語](native_jp.md) 

# 箱庭 ROS シミュレータ

[TOPPERSプロジェクト箱庭WG](https://toppers.github.io/hakoniwa)では，IoT／クラウドロボティクス時代の仮想シミュレーション環境である『箱庭』の研究開発を進めています．

本リポジトリでは，箱庭上で ROS 2 プログラムのシミュレーションを簡単にお試しできる環境を公開しています．

本ページでは，ネイティブのLinux環境（WSL2含む）での動作手順を示します．Docker 環境での利用手順は [README.md](/README.md) をご参照ください．

## 想定する PC 環境

* Windows 環境: Windows 10/11 with Ubuntu 20.04 LTS on WSL2/WSLg
* Linux 環境: Ubuntu 20.04 LTS

## PC環境の準備

### ROS 2 Foxy のインストール

次のページに従って ROS 2 Foxy をインストールしてください．

https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

### 箱庭環境のインストール

必要なパッケージをインストールします．

```
sudo apt-get update
sudo apt-get install -y g++ python3-colcon-common-extensions
```

### Unity 環境

* Unity Hub 3.1.1 以降
* Unity Editor 2021.3.0f1
  * Unity Hub の「Installs > Install Editor」画面に本バージョンが表示されない場合は，[Unity Dowonload Archive](https://unity3d.com/get-unity/download/archive) の本バージョンの "Unity Hub" をクリックしてインストールできます． 

### 本リポジトリのclone

Windows 環境では，操作は全てWSL2/Linuxのシェル上で行います．WSL2のファイルシステム配下（`/home/${USER}/`以下）ではなくWindowsファイルシステム配下（`/mnt/c/`以下）で実行してください．

ターミナルで下記を実行して本リポジトリをcloneしてください．

```
git clone --recursive https://github.com/toppers/hakoniwa-ros2sim.git
```

## シミュレータの導入手順

ターミナルを2個起動します（以降の説明では，ターミナルAおよびターミナルBと呼びます）．

ターミナルAでROS 2のワークスペースに移動し，箱庭のROS環境をインストールします．

```
cd ros2/workspace
bash hako-install.bash
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

### ターミナルAでの操作

ターミナルAでROS-TCP-ENDPOINTを起動しましょう．

```
cd ros2/workspace
bash launch.bash
```

### ターミナルBでの操作

ターミナルBでROS2プログラムを起動しましょう．

```
cd ros2/workspace
bash run.bash tb3 TB3RoboModel
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

## 謝辞
* TurtleBot3 の Unity パッケージの設計と作成にあたっては，宝塚大学 東京メディア芸術学部 吉岡章夫准教授および学部生の杉崎涼志さん，木村明美さんにご協力いただきました．
* TurtleBot3 のUnity アセットは，株式会社ロボティズ様より提供いただいたデータを基に作成しています．ご協力いただき深く感謝いたします．

## ライセンス

[TOPPERSライセンス](https://www.toppers.jp/license.html)で公開しています．  
著作権者はTOPPERSプロジェクト箱庭ワーキンググループです．詳細は[LICENSE.md](./LICENSE.md)をご参照ください．
