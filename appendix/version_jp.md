[English](version.md) | [日本語](version_jp.md) 

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
### v1.1.4 @ 2022/11/25

* Fix json files by by s-hosoai in https://github.com/toppers/hakoniwa-ros2sim/pull/45
* mod instruction to install docker by the script onto hakoniwa-single_robot repository by @takasehideki in https://github.com/toppers/hakoniwa-ros2sim/pull/44

### v1.1.3 @ 2022/10/03

* add hakoniwa proxy for ros2 by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/34
* hakoniwa cpp core function is added by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/36
* Englishization of README by @urashima0429 in https://github.com/toppers/hakoniwa-ros2sim/pull/37
* Add EV3 Robot Model by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/38
* 電車モデルできました！ by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/39
* add camera mover on main camera by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/40 

- 箱庭アセットとバージョン (commit hash) 情報
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [c60ec37e95e3b33e4f09881525cd3eabbea1f781](https://github.com/toppers/hakoniwa-core/tree/c60ec37e95e3b33e4f09881525cd3eabbea1f781)

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
