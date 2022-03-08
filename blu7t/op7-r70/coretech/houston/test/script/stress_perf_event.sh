adb shell setenforce 0
adb shell "echo 4-6 > /dev/cpuset/top-app/cpus"


# case 1
cnt=100
while [ $cnt -ne 0 ];
do
adb shell "am start -n com.tencent.tmgp.sgame/.SGameActivity"
sleep 3
pid=$(adb shell ps  | grep -v xg_service | awk '/tencent/{print $2}')
echo "cnt $cnt pid $pid"
adb shell "echo '$pid' > /sys/module/houston/parameters/perf_ready"
adb shell aisdebug resume $pid com.tencent.tmgp.sgame
sleep 3
adb shell aisdebug set_dumpfile true
sleep 10
adb shell kill -9 $pid
sleep 3
((--cnt))
done

