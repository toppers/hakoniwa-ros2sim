# バージョン情報・更新履歴

## バージョン番号の付与規則

`vX.Y.Z` のようにバージョン番号を付与します．この番号は `appendix/latest_version.txt` で管理されています．また，[Git/GitHubのtag/release](https://github.com/toppers/hakoniwa-ros2sim/releases)および[Docker Hubのtag番号](https://hub.docker.com/r/toppersjp/hakoniwa-ros2sim/tags)に対応しています．

各バージョン番号の更新は，次の規則で行います．

- Major version (`X`): 
    - 箱庭コア技術の新機能の導入
    - 使用方法の大きな変更　など
- Minor version (`Y`): 
    - 箱庭アセット技術（各リポジトリ）の更新
    - 大幅な機能の修正
    - シミュレータ本体の新たな機能の追加　など
- Revision (`Z`): 
    - 軽微な機能の修正・バグの対応
    - ドキュメントの更新　など

## 更新履歴

### v1.1.2 @ 2022/05/14

* add robot arm stick parts for general purpose (fix #25) by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/26
* specify version on `git clone` by @takasehideki in https://github.com/toppers/hakoniwa-ros2sim/pull/29
* add robot arm (fix #27) by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/28
* improve README to make it easier to copy command by @takasehideki in https://github.com/toppers/hakoniwa-ros2sim/pull/30
* Refactor tb3ctrl by @s-hosoai in https://github.com/toppers/hakoniwa-ros2sim/pull/32

- 箱庭アセットとバージョン (commit hash) 情報
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [35c47fe42af58f37a4843e5e789e5f749acfbf0b](https://github.com/toppers/hakoniwa-core/tree/35c47fe42af58f37a4843e5e789e5f749acfbf0b)

### v1.1.1 @ 2022/05/06

- ネイティブのLinux環境（WSL2含む）での動作手順の追加
- Docker image の改善
- 箱庭アセットとバージョン (commit hash) 情報
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [35c47fe42af58f37a4843e5e789e5f749acfbf0b](https://github.com/toppers/hakoniwa-core/tree/35c47fe42af58f37a4843e5e789e5f749acfbf0b)

### v1.1.0 @ 2022/05/06

- リポジトリ名の変更
- README の作成と改善
- Docker image の公開
- ユーザ利便性の改善
    - submodule の URL を https に変更
    - Docker 操作のスクリプトを全プラットフォームで共通化
- 箱庭アセットとバージョン (commit hash) 情報
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [35c47fe42af58f37a4843e5e789e5f749acfbf0b](https://github.com/toppers/hakoniwa-core/tree/35c47fe42af58f37a4843e5e789e5f749acfbf0b)

### v1.0.0 @ 2022/04/23

- 最初のリリース
- 箱庭アセットとバージョン (commit hash) 情報
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [2628fdbed6f49f44bc988e65b08858bf67424b79](https://github.com/toppers/hakoniwa-core/tree/2628fdbed6f49f44bc988e65b08858bf67424b79)
