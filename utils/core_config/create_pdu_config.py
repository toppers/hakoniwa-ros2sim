#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
import os
from collections import OrderedDict

if len(sys.argv) != 5:
	print "Usage: " + sys.argv[0] + " <in_dir> <ros_msg_list> <out_dir> <custom>"
	sys.exit()

in_dir=sys.argv[1]
in_file=sys.argv[2]
out_dir=sys.argv[3]


custom=None
custom_file=sys.argv[4]
if os.path.exists(custom_file):
	tmp_file = open(custom_file)
	custom = json.load(tmp_file)

out_filename='pdu_configs.json'

container = list()
for pkg_msg in open(in_file, 'r'):
    entry = OrderedDict()
    entry['pdu_type_name'] = pkg_msg.strip()
    entry['pdu_data_field_path'] = in_dir + '/' + pkg_msg.strip() + '.json'
    container.append(entry)
    entry = OrderedDict()
    entry['pdu_type_name'] = pkg_msg.strip().split('/')[1]
    entry['pdu_data_field_path'] = in_dir + '/' + pkg_msg.strip() + '.json'
    container.append(entry)
    

entry = OrderedDict()
entry['pdu_type_name'] = 'time'
entry['pdu_data_field_path'] = in_dir + '/builtin_interfaces/Time.json'
container.append(entry)

if custom != None:
    entry = OrderedDict()
    entry['pdu_type_name'] = 'HakoniwaSimTime'
    entry['pdu_data_field_path'] = './HakoniwaSimTime.json'
    container.append(entry)


with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)


