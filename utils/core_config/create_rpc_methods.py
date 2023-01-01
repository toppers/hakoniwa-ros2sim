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
out_filename='rpc_methods.json'

file = open(in_file)
custom = json.load(file)

container = list()
for robo in custom['robots']:
    if robo.get('rpc_pdu_readers') != None:
      for p in robo['rpc_pdu_readers']:
        entry = OrderedDict()
        entry['method_name'] = hakoniwa_utils.get_custom_rpc_reader_method_name(robo)
        entry['asset_name'] = 'UnityAssetRpc'
        entry['channel_id'] = p['channel_id']
        entry['pdu_size'] = p['pdu_size']
        entry['is_read'] = True
        container.append(entry)
    if robo.get('rpc_pdu_writers') != None:
      for p in robo['rpc_pdu_writers']:
        entry = OrderedDict()
        entry['method_name'] = hakoniwa_utils.get_custom_rpc_writer_method_name(robo)
        entry['asset_name'] = 'UnityAssetRpc'
        entry['channel_id'] = p['channel_id']
        entry['pdu_size'] = p['pdu_size']
        entry['is_read'] = False
        container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

