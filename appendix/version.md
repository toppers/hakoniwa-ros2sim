# バージョン情報・更新履歴

## バージョン番号の付与規則

`vX.Y.Z` のようにバージョン番号を付与します．この番号は `docker/image_name.txt` でも管理されています．また，[Git/GitHubのtag/release](https://github.com/toppers/hakoniwa-ros2sim/releases)および[Docker Hubのtag番号](https://hub.docker.com/r/toppersjp/hakoniwa-ros2sim/tags)に対応しています．

各バージョン番号の更新は，次の規則で行います．

- Major version (`X`): 
    - 箱庭コア技術の新機能の導入
    - 使用方法の大きな変更　など
- Minor version (`Y`): 
    - 箱庭アセット技術（各リポジトリ）の更新
    - 大幅な機能の修正
    - シミュレータ本体の新たな機能の追加　など
- Revison (`Z`): 
    - 軽微な機能の修正・バグの対応
    - ドキュメントの更新　など

## 更新履歴

### v1.0.0 @ 2022/04/23

- 最初のリリース
- 箱庭アセットとバージョン (commit hash) 情報
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [2628fdbed6f49f44bc988e65b08858bf67424b79](https://github.com/toppers/hakoniwa-core/tree/2628fdbed6f49f44bc988e65b08858bf67424b79)
