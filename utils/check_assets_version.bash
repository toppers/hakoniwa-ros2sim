#!/bin/bash

assets=("hakoniwa-core")

for ast in ${assets[@]}; do
    cd /root/workspace/${ast}
    sha=`git rev-parse HEAD`
    echo "    - [${ast}](https://github.com/toppers/${ast}) / sha: [${sha}](https://github.com/toppers/${ast}/tree/${sha})"
done

