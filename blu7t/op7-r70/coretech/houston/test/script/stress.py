#!/usr/bin/env python2.7
import time
import os
import subprocess
from subprocess import Popen, PIPE
import random
from datetime import datetime
import pandas

cnt = 1

def process_csv(file_name):
	print('process file %s' % file_name)
	data = pandas.read_csv(file_name)
	data['real'] = data['cycle'] * 1000000 / data['CLUS1 freq']
	tag = 'schedstat       '
	if tag in list(data):
		data['t1'] = data[tag]
	else:
		data['t1'] = data['schedstat']
	data['t1_%'] = data['t1'] / data['real']
	data['t1_m'] = data['t1_%'].mean()
	data['t1_s'] = data['t1_%'].std()

	#data['t2'] = data['cachemiss_L2']
	#data['t2_%'] = data['t2'] / data['real']
	#data['t2_m'] = data['t2_%'].mean()
	#data['t2_s'] = data['t2_%'].std()

	#print("status: t1_m: %f t1_s:%f t2_m:%f t2_s:%f" % (data['t1_m'][0], data['t1_s'][0], data['t2_m'][0], data['t2_s'][0]))

	print("status: t1_m: %f t1_s:%f" % (data['t1_m'][0], data['t1_s'][0]))
	data.to_csv(file_name, index=False)
	print('file processed')

def print_sleep(sec):
	while sec >= 0:
		print(','),
		time.sleep(1)
		sec -= 1

while True:
	print('test %d' % cnt)
	print('waiting for device')
	loop = True
	while loop:
		process = Popen(['lsusb'], stdout=PIPE)
		output = process.communicate()
		#print(output)
		for line in output:
			if line and line.find('Qual') >= 0:
				loop = False
				print('device is ready')
				break

	print('screen unlock')
	os.system('adb shell input keyevent 82')

	print('set top app to cpu 4-6')
	os.system('adb shell \"echo 4-6 > /dev/cpuset/top-app/cpus \"')

	print('lock cpu/ddr freq');
	subprocess.call(['adb', 'shell', 'echo 1632000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq'])
	subprocess.call(['adb', 'shell', 'echo 1632000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq'])
	subprocess.call(['adb', 'shell', 'echo 1632000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq'])
	subprocess.call(['adb', 'shell', 'echo 1632000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq'])
	subprocess.call(['adb', 'shell', 'echo 710400 > /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq'])
	subprocess.call(['adb', 'shell', 'echo 710400 > /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq'])
	subprocess.call(['adb', 'shell', 'echo 710400 > /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq'])
	subprocess.call(['adb', 'shell', 'echo 710400 > /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq'])
	subprocess.call(['adb', 'shell', 'echo 2419200 > /sys/devices/system/cpu/cpu7/cpufreq/scaling_max_freq'])
	subprocess.call(['adb', 'shell', 'echo 2419200 > /sys/devices/system/cpu/cpu7/cpufreq/scaling_min_freq'])
	subprocess.call(['adb', 'shell', 'echo 2419200 > /sys/devices/system/cpu/cpu7/cpufreq/scaling_max_freq'])
	subprocess.call(['adb', 'shell', 'echo 2419200 > /sys/devices/system/cpu/cpu7/cpufreq/scaling_min_freq'])

	subprocess.call(['adb', 'shell', 'echo {class: ddr, res: fixed, val: 681} > /sys/kernel/debug/aop_send_message'])

	print_sleep(5)
	print('start activity')
	os.system('adb shell \"am start -n com.tencent.tmgp.sgame/.SGameActivity \"')

	print_sleep(5)
	print('get target_pid')
	process = Popen(['adb', 'shell', 'ps -A | grep tencent |  grep -v com.tencent.tmgp.sgame:xg_service_v3'], stdout=PIPE)
	(output, err) = process.communicate()
	#print(output)
	target_pid = output.split()[1]
	#print(target_pid)
	if not target_pid:
		print('don\'t get target pid')
		exit()

	r = random.randint(4,7)
	print('wait for %d sec' % r)
	print_sleep(r)
	print('echo target_pid %s to monitor' % target_pid)
	cmd = 'echo ' + target_pid + ' > /sys/module/houston/parameters/perf_ready'
	process = Popen(['adb', 'shell', cmd], stdout=PIPE)
	process = Popen(['adb', 'shell', 'cat  /sys/module/houston/parameters/perf_ready'], stdout=PIPE)
	(output, err) = process.communicate()
	if output and output.rstrip() != target_pid:
		print('echo error')
		exit()
	print('collect data')
	cmd = 'adb shell datacollector 1000'
	subprocess.call(cmd.split())
	#output = str(datetime.now()).replace(' ', '_') + "_" + str(cnt) + ".csv"
	output = str(cnt).zfill(4) + ".csv"
	cmd = 'adb pull /data/test.csv ' + output
	subprocess.call(cmd.split())
	process_csv(output)
	print_sleep(3)
	os.system('adb reboot')
	print_sleep(60)
	cnt += 1
