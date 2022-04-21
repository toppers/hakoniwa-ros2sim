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

def get_pdu_name(e):
	return e['topic_message_name'] + 'Pdu'

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
		