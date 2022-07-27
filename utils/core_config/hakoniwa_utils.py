#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re


def get_robolist(ros_topics):
	tmp_list = list()
	for e in ros_topics['fields']:
		tmp_list.append(e['robot_name'])
	return list(set(tmp_list))

def get_proxylist(custom):
	tmp_list = list()
	for e in custom['proxies']:
		tmp_list.append(e['robot_name'])
	return list(set(tmp_list))


def get_entry(ros_topics, type, name):
	tmp_list = list()
	for e in ros_topics['fields']:
		if e[type] == name:
			tmp_list.append( e )
	return tmp_list

def is_exit_robot(micon_robots, name):
	if micon_robots == None:
		return False
	for e in micon_robots['robots']:
		if e['name'] == name:
			return True
	return False

def get_pdu_name(e):
	return e['topic_message_name'] + 'Pdu'

def get_custom_asset_name(e):
	return 'Athrill'
	# For now, the name is Athrill only.
	# The below name will be accepted when the single-robot 
	# device_config.txt template is updated. 
	# return e['name'] + "_" + e['udp_proxy']['name'] + 'Proxy'

def get_custom_pdu_name(e):
	return e['name'] + 'Pdu'

def get_custom_udp_reader_method_name(e):
	return e['name'] + 'UdpReader'

def get_custom_udp_writer_method_name(e):
	return e['name'] + 'UdpWriter'

def get_pdu_type_name(e):
	return e['topic_type_name']

def get_pdu_topic_name(e):
	return e['topic_message_name']

def get_pdu_class_name(rw_type):
	if rw_type == 'w':
		return 'Hakoniwa.PluggableAsset.Communication.Pdu.ROS.RosTopicPduWriter'
	else:
		return'Hakoniwa.PluggableAsset.Communication.Pdu.ROS.RosTopicPduReader'

def get_pdu_conv_class_name(rw_type, pkg_name):
	if rw_type == 'w':
		return 'Hakoniwa.PluggableAsset.Communication.Pdu.ROS.' + pkg_name + '.RosTopicPduWriterConverter'
	else:
		return 'Hakoniwa.PluggableAsset.Communication.Pdu.ROS.' + pkg_name + '.RosTopicPduReaderConverter'
		