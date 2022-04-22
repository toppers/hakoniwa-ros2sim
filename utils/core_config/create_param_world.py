#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import json
import sys
from collections import OrderedDict

if len(sys.argv) != 3:
    print "Usage: " + sys.argv[0] + " <scale> <out_dir>"
    sys.exit()


scale = float(sys.argv[1])
out_dir = sys.argv[2]

out_data = OrderedDict()

out_data['scale'] = OrderedDict()
out_data['scale']['scan'] = scale
out_data['scale']['odom'] = scale
out_data['scale']['cmdvel'] = scale

with open(out_dir + '/param_world.json', mode='wt') as out_file:
    json.dump(out_data, out_file, ensure_ascii=False, indent=2)
