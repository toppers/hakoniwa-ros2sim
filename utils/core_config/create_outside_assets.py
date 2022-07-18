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
for robo in custom['robots']:
	entry = OrderedDict()
	entry['name'] = hakoniwa_utils.get_custom_asset_name(robo)
	entry['class_name'] = robo['udp_proxy']['class_name']
	container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

