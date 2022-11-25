[English](version.md) | [日本語](version_jp.md) 

# Version & Updates

## Rules of version numbering

A version number is assigned, such as `vX.Y.Z`.
This number is managed in  `appendix/latest_version.txt` .
In addition, [Git/GitHub tag/release](https://github.com/toppers/hakoniwa-ros2sim/releases) and [Docker Hub tag number](https://hub.docker.com/r/toppersjp/hakoniwa-ros2sim/tags)．

Each version number is updated according to the following rules.

- Major version (`X`): 
    - Introduction of new features in the core technology of Hakoniwa
    - Major changes in usage, etc.
- Minor version (`Y`): 
    - Update of Hakoniwa asset technology (repositories)
    - Major feature fixes
    - New features in the simulator core, etc.
- Revision (`Z`): 
    - Minor fixes, bug fixes, etc.
    - Documentation updates, etc.

## Update History

### v1.1.4 @ 2022/11/25

* Fix json files by by s-hosoai in https://github.com/toppers/hakoniwa-ros2sim/pull/45
* mod instruction to install docker by the script onto hakoniwa-single_robot repository by @takasehideki in https://github.com/toppers/hakoniwa-ros2sim/pull/44

### v1.1.3 @ 2022/10/03

* add hakoniwa proxy for ros2 by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/34
* hakoniwa cpp core function is added by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/36
* Englishization of README by @urashima0429 in https://github.com/toppers/hakoniwa-ros2sim/pull/37
* Add EV3 Robot Model by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/38
* Add Train & Signal Model by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/39
* add camera mover on main camera by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/40

- Hakoniwa asset and version (commit hash) information 
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [c60ec37e95e3b33e4f09881525cd3eabbea1f781](https://github.com/toppers/hakoniwa-core/tree/c60ec37e95e3b33e4f09881525cd3eabbea1f781)

### v1.1.2 @ 2022/05/14

* add robot arm stick parts for general purpose (fix #25) by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/26
* specify version on `git clone` by @takasehideki in https://github.com/toppers/hakoniwa-ros2sim/pull/29
* add robot arm (fix #27) by @tmori in https://github.com/toppers/hakoniwa-ros2sim/pull/28
* improve README to make it easier to copy command by @takasehideki in https://github.com/toppers/hakoniwa-ros2sim/pull/30
* Refactor tb3ctrl by @s-hosoai in https://github.com/toppers/hakoniwa-ros2sim/pull/32

- Hakoniwa asset and version (commit hash) information 
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [35c47fe42af58f37a4843e5e789e5f749acfbf0b](https://github.com/toppers/hakoniwa-core/tree/35c47fe42af58f37a4843e5e789e5f749acfbf0b)

### v1.1.1 @ 2022/05/06

- Add procedures for operating in native Linux environments (including WSL2)
- Improvements to Docker image
- Hakoniwa asset and version (commit hash) information 
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [35c47fe42af58f37a4843e5e789e5f749acfbf0b](https://github.com/toppers/hakoniwa-core/tree/35c47fe42af58f37a4843e5e789e5f749acfbf0b)

### v1.1.0 @ 2022/05/06

- Change Repository Name
- Create and improve README
- Release of Docker image
- Improve usability
    - Change submodule URL to https
    - Unify Docker operation scripts across all platforms
- Hakoniwa asset and version (commit hash) information 
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [35c47fe42af58f37a4843e5e789e5f749acfbf0b](https://github.com/toppers/hakoniwa-core/tree/35c47fe42af58f37a4843e5e789e5f749acfbf0b)

### v1.0.0 @ 2022/04/23

- First release
- Hakoniwa asset and version (commit hash) information 
    - [hakoniwa-core](https://github.com/toppers/hakoniwa-core) / sha: [2628fdbed6f49f44bc988e65b08858bf67424b79](https://github.com/toppers/hakoniwa-core/tree/2628fdbed6f49f44bc988e65b08858bf67424b79)
