### Docker 環境の導入
Docker Engineがインストールされている必要があります．Linuxのターミナルで下記のコマンドの結果が同じように出力されていれば，すでにインストール済みです（`$`から始まる行は実行するコマンドを示しています）．

```
$ which docker
/usr/bin/docker
$ service --status-all |& grep docker
 [ + ]  docker   # または " [ - ]  docker "
$ service docker status
 * Docker is running   # または " * Docker is not running "
```

Docker Engineのインストールはやや手数が多いため，下記の公式マニュアルにある実行コマンドを["toppersjp/hakoniwa-single_robot リポジトリの `docker/install-docker.bash`](https://github.com/toppers/hakoniwa-single_robot/blob/main/docker/install-docker.bash) 
にまとめています．本スクリプトの実行時に問題がありましたら，公式マニュアルの手順に従ってインストールを進めてください．  
[Install Docker Engine on Ubuntu | Docker Documentation](https://docs.docker.com/engine/install/ubuntu/)

スクリプトを用いたDockerのインストールは，下記のように実行してください．

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