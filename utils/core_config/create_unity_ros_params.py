#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

if len(sys.argv) != 3:
	print "Usage: " + sys.argv[0] + " <ipaddr> <out_dir>"
	sys.exit()


ipaddr=sys.argv[1]
out_dir=sys.argv[2]

out_data = OrderedDict()

out_data['connection_startup'] = True
out_data['ip_address'] = ipaddr
out_data['portno'] = 10000
out_data['show_hud'] = False
out_data['keep_alive_time'] = 10
out_data['network_timeout'] = 2
out_data['sleep_time'] = 0.01

with open(out_dir + '/unity_ros_params.json', mode='wt') as out_file:
  json.dump(out_data, out_file, ensure_ascii=False, indent=2)

