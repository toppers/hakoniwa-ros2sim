#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

import hakoniwa_utils

if len(sys.argv) != 3:
	print "Usage: " + sys.argv[0] + " <custom> <out_dir>"
	sys.exit()

in_file=sys.argv[1]
out_dir=sys.argv[2]
out_filename='outside_assets.json'

file = open(in_file)
custom = json.load(file)


container = list()
for robo in custom['proxies']:
	entry = OrderedDict()
	entry['name'] = robo['robot_name'] + 'Proxy'
	entry['class_name'] ='Hakoniwa.PluggableAsset.Assets.Micon.SimpleMiconAssetController'
	entry['path'] = './MiconPdu.dll'
	container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

