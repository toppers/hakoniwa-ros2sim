#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

import hakoniwa_utils

if len(sys.argv) != 4:
	print "Usage: " + sys.argv[0] + " <custom> <core_ipaddr> <out_dir>"
	sys.exit()

in_file=sys.argv[1]
core_ipaddr=sys.argv[2]
out_dir=sys.argv[3]
out_filename='udp_methods.json'

file = open(in_file)
custom = json.load(file)


container = list()
for robo in custom['robots']:
	## Robo
	#WRITER
	entry = OrderedDict()
	entry['method_name'] = hakoniwa_utils.get_custom_udp_writer_method_name(robo)
	entry['ipaddr'] = robo['udp_proxy']['ipaddr']
	entry['portno'] = robo['udp_proxy']['rx_port']
	entry['iosize'] = 1024
	entry['is_read'] = False
	container.append(entry)

	#READER
	entry = OrderedDict()
	entry['method_name'] = hakoniwa_utils.get_custom_udp_reader_method_name(robo)
	entry['ipaddr'] = core_ipaddr
	entry['portno'] = robo['udp_proxy']['tx_port']
	entry['iosize'] = 1024
	entry['is_read'] = True
	container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

