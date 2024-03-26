[English](README.md) | [日本語](README_jp.md) 

# 箱庭 ROS シミュレータ

[TOPPERSプロジェクト箱庭WG](https://toppers.github.io/hakoniwa)では，IoT／クラウドロボティクス時代の仮想シミュレーション環境である『箱庭』の研究開発を進めています．

本リポジトリでは，箱庭上で ROS 2 プログラムのシミュレーションを簡単にお試しできる環境を公開しています．


## 想定する PC 環境
* Linux 環境: Ubuntu 22.04 LTS

## PC環境の準備

本シミュレータではネイティブのUbuntu環境 または Docker を利用します．

Docker環境での動作手順は [appendix/docker_jp.md](/appendix/docker_jp.md) をご参照ください．

### Unity 環境のインストール

* Unity Hub 3.5.2 以降
* Unity Editor 2021.3.17f1
  * Unity Hub の「Installs > Install Editor」画面に本バージョンが表示されない場合は，[Unity Dowonload Archive](https://unity3d.com/get-unity/download/archive) の本バージョンの "Unity Hub" をクリックしてインストールできます． 

### 本リポジトリのclone

現在の最新版は **v1.2.0** です．
v1.1.*以前の環境とは互換性がなく、操作方法も異なります。またv1.2.0はUbuntu環境だけのサポートとなります。Windows（WSL）やMacには対応していません。

「[バージョン情報・更新履歴](/appendix/version.md)」も参照してください（バージョン番号は[Git/GitHubのtag/release](https://github.com/toppers/hakoniwa-ros2sim/releases)および[Docker Hubのtag番号](https://hub.docker.com/r/toppersjp/hakoniwa-ros2sim/tags)に対応しています）

ターミナルで下記を実行して本リポジトリをcloneしてください．

```
git clone --recursive -b v1.2.0 https://github.com/toppers/hakoniwa-ros2sim.git
```
以降、ユーザのホームディレクトリにcloneしたものとして説明します。
異なるディレクトリにした場合は適宜、初段のパスを読み替えてください。

## シミュレータの導入手順
### hakoniwa-core-cpp-clientのビルド
箱庭コア機能のライブラリをビルドします。ここで作成したライブラリは、conductorやros2pdu、Unityのプロジェクトで利用します。
```
cd ~/hakoniwa-ros2sim/hakoniwa-core-cpp-client
bash build.bash
bash install.bash
```

### hakoniwa-conductorのビルド
箱庭アセット間の調停を行うconductorをビルドします。
```
cd ~/hakoniwa-ros2sim/hakoniwa-conductor/main
bash build.bash
sudo bash install.bash
```

### hakoniwa-ros2pduのビルド
ROS 2と箱庭内の通信の変換を行うros2pduをビルドします。
ビルドにはROS 2環境が必要です。
```
source /opt/ros/humble/setup.bash
cd ~/hakoniwa-ros2sim/hakoniwa-ros2pdu
bash create_all_pdus.bash
bash create_proxy.bash ./config/custom.json
cd workspace
bash build.bash
source install/local_setup.bash
```

### Unity プロジェクトの準備
Unityで用いるロボットのモデルを準備します。
以下コマンドでロボットのモデルやライブラリがダウンロードされます。
```
cd hakoniwa-ros2sim/hakoniwa-unity-tb3model
bash install.bash
```

先ほど作成したhakoniwa-core-cpp-clientのライブラリを既存のものと置き換えます。
```
cp hakoniwa-core-cpp-client/cmake-build/src/hakoc/libshakoc.so hakoniwa-unity-tb3model/plugin/plugin-srcs/Assets/Plugin/Libs/
```
これでUnityプロジェクトの準備が完了しました。

### Unity プロジェクトを開く

Unity Hubを起動し，右上の「開く」をクリックして、先ほどクローンしたROS対応版箱庭ソース上の以下のディレクトリを指定します．

場所：hakoniwa-ros2sim/hakoniwa-unity-tb3model/plugin/plugin-srcs

<!-- ![](https://camo.qiitausercontent.com/34b3dee89c42787380e888af0bb4a321c866ebe3/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f39353336626137632d383932372d383037302d613934372d3361643839396436363731622e706e67) -->

起動時にエラーが出るため、ダイアログに従いSafemodeで入ります。

Unityの上部メニューより，Edit / Project Settingsを開き．左のツリーからPlayerを選択し，
右に表示される項目の最下部付近のScript CompilationのScripting Define Symbolsの箇所で，
＋を押し項目を追加し、「NO_USE_GRPC」を記入します。その後すぐ下のApplyで設定を反映します。
ダイアログを閉じると再ビルドされ，エラーが表示されなくなれば成功です。

起動すると，以下の画面が表示されます．

<!-- 画像はあとで差し替え
![](https://camo.qiitausercontent.com/118f0af1ef4fdf3cdb4c2e7017d7ac92e2079943/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f33303965666239392d643663612d383937612d356233342d3334386233333763353339322e706e67) -->


この状態で、画面左下にある「プロジェクトビュー」の「Scene / TB3 」を選択すると、
画面下に「Hakoniwa」というシーンがあり，これをダブルクリックするとエディタ上にロボットとコースが表示されます。

## シミュレータの実行手順

### 制御プログラムおよびConductorの実行
```
cd ~/hakoniwa-ros2sim/workspace/runtime
bash run.bash
```

### Unityのシミュレーションの開始
Unityのシミュレーション開始ボタンをクリックすると，以下の起動画面が出てきます．

<!-- ![](https://camo.qiitausercontent.com/8aa80400f8a6b9527febde6edc5778187dc5f1cd/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f64396265303530612d393662352d353032322d653230642d3964616332633166613430322e706e67) -->


この状態で，Unity側の「開始」ボタンをクリックすると箱庭のシミュレーション開始し，TurtleBot3が動き出します．

### 動作例

<!-- ![動作例](https://camo.qiitausercontent.com/6aae22e5ac3d57f9faaf43c75d9eee84eb0d0dc8/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e61702d6e6f727468656173742d312e616d617a6f6e6177732e636f6d2f302f3234343134372f32316433333837622d336663652d303430322d623734362d3666353333353763643239612e676966) -->

この動画の各ウィンドウは，それぞれ次の通り対応しています．

- 右：ターミナルA
- 左：Unity

## Contributing

本リポジトリで公開している「箱庭 ROS シミュレータ」について，ご意見や改善の提案などをぜひ [こちらのGitHub Discussions](https://github.com/toppers/hakoniwa/discussions/categories/idea-request) でお知らせください．改修提案の [Pull Requests](https://github.com/toppers/hakoniwa-ros2sim/pulls) も歓迎いたします．

## 謝辞
* TurtleBot3 の Unity パッケージの設計と作成にあたっては，宝塚大学 東京メディア芸術学部 吉岡章夫准教授および学部生の杉崎涼志さん，木村明美さんにご協力いただきました．
* TurtleBot3 のUnity アセットは，株式会社ロボティズ様より提供いただいたデータを基に作成しています．ご協力いただき深く感謝いたします．

## ライセンス

[TOPPERSライセンス](https://www.toppers.jp/license.html)で公開しています．  
著作権者はTOPPERSプロジェクト箱庭ワーキンググループです．詳細は[LICENSE.md](./LICENSE.md)をご参照ください．
