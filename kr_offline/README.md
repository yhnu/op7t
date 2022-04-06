# kr_offline工具使用说明

前段时间做了krhook模块(在内核层进行用户层堆栈回溯), 已经通过日志打印出了maps信息和pc调用链, 为了达到类似安卓崩溃分析的效果, 我们还需要一个离线工具,具体原理可以参考

https://support.unity.com/hc/en-us/articles/115000292166-Symbolicate-Android-crash

## 先简单讲讲krhook模块的优点

krhook实现了拦截系统调用并进行用户层堆栈回溯的功能, 所有操作均在内核层进行处理, 可以过所有的防护系统

## 阅读须知.阅读须知.阅读须知.

因为内核层兼容比较复杂, 如果遇到对应问题, 先查看环境是否一致(内核版本需要为4.14.117且自己做过修改, 对应系统为安卓10)

## 自己先写一个APP方便理解对应的原理

对应示例代码

```c#
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class FileOpenTest : MonoBehaviour
{
    public void FileOpen()
    {
        byte[] data = { 1, 2, 3, 4 };
        // below it will call openat and tigger krhook for stack walk
        var fileStream = File.Open(Path.Combine(Application.persistentDataPath, "a.txt"), FileMode.OpenOrCreate);
        fileStream.Write(data, 0, data.Length);
        fileStream.Close();
    }
}
```

如果你想简单那就用下面的apk即可

[https://github.com/yhnu/op7t/releases/download/v1.0/demo.apk](https://github.com/yhnu/op7t/releases/download/v1.0/demo.apk)


安装对应的apk,获取uid为10226

```shell
OnePlus7T:/data/local/tmp # ps -ef |grep krh
u0_a226      23140   757 2 23:11:53 ?     00:11:41 com.DefaultCompany.krhook_unity3d
root         26124  7633 3 11:40:54 pts/1 00:00:00 grep krh
OnePlus7T:/data/local/tmp #
```

## 启动krhook模块, 并开启uid过滤(这就是为什么上面需要拿到uid的原因, 另外如果app不卸载, uid并不会变化)

```shell
OnePlus7T:/data/local/tmp # insmod krhook.ko
OnePlus7T:/data/local/tmp # lsmod
Module                  Size  Used by
krhook                 24576  0
wlan                 7323648  0
rmnet_perf             36864  0
OnePlus7T:/data/local/tmp # echo 10226 > /dev/mypid
OnePlus7T:/data/local/tmp #
```

## 点击apk对应的按钮, 触发openat调用
```shell
OnePlus7T:/data/local/tmp # dmesg
[63479.878705] [20210929_08:59:12.888466]@5 [s]openat /storage/emulated/0/Android/data/com.DefaultCompany.krhook_unity3d/files/a.txt
......
[63479.879722] [20210929_08:59:12.889486]@5 [m]0x703dd51000-0x7040000000 0x00000000 r--p /data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so
[63479.879730] [20210929_08:59:12.889494]@5 [m]0x7071b4e000-0x7071c6c000 0x00000000 r--p /apex/com.android.runtime/lib64/bionic/libc.so
[63479.879735] [20210929_08:59:12.889499]@5 [m]0x7071c6c000-0x7075000000 0x00000000 r--p /data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libunity.so
[63479.879741] [20210929_08:59:12.889506]@5 [m]0x7079bee000-0x707bbee000 rw-s
[63479.879746] [20210929_08:59:12.889511]@5 [m]0x707dbee000-0x707e5f7000 rw-s
[63479.879751] [20210929_08:59:12.889515]@5 [m]0x707e5f7000-0x707f000000 rw-s
[63479.879757] [20210929_08:59:12.889521]@5 [m]0x70a035c000-0x70a0d65000 rw-s
[63479.879762] [20210929_08:59:12.889526]@5 [m]0x70a0d65000-0x70a0ec9000 0x00000000 r--p /vendor/lib64/hw/vulkan.msmnile.so
[63479.879766] [20210929_08:59:12.889531]@5 [m]0x70a0ec9000-0x70a0fd7000 0x00164000 --xp /vendor/lib64/hw/vulkan.msmnile.so
[63479.879770] [20210929_08:59:12.889535]@5 [m]0x70a0fd7000-0x70a0fe7000 0x00272000 rw-p /vendor/lib64/hw/vulkan.msmnile.so
[63479.879774] [20210929_08:59:12.889539]@5 [m]0x70a0fe7000-0x70a0fef000 0x00282000 r--p /vendor/lib64/hw/vulkan.msmnile.so
[63479.879800] [20210929_08:59:12.889564]@5 [m]0x7184a03000-0x7185d88000 0x00000000 r-xp /data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so
......
[63480.310487] [20210929_08:59:13.320249]@5 [e]openat [/storage/emulated/0/Android/data/com.DefaultCompany.krhook_unity3d/files/a.txt] current->pid:[23205] ppid:[757] uid:[10226] tgid:[23140] stack:0x7277206388|0x7184f09c18|0x7184f2b568|0x718597e054|0x7185971930|0x718596fa6c|0x718596b93c|0x718596b8b0|0x718599e918|0x718548ffd8|0x718549c290|0x718549f61c|0x71852ed8b0|0x71852ed980|0x7184ded750|0x71852d80b8|0x71855edd38|0x71857c53fc|0x7184df0ab0|0x71852e7388|0x71852e583c|0x71852e50cc|0x7184d4f84c|0x71852d67e4|0x7184d6b230|0x7184f9feac|0x7184f02f04|0x718a4341d0|0xffffffffffffffff|
```

将上面的日志信息完整的保存下来

[https://github.com/yhnu/op7t/blob/dev/kr_offline/demo.krhook.txt](https://github.com/yhnu/op7t/blob/dev/kr_offline/demo.krhook.txt)

## 堆栈还原

需要借助离线工具

[https://github.com/yhnu/op7t/releases/download/v1.0/kr_offline.exe](https://github.com/yhnu/op7t/releases/download/v1.0/kr_offline.exe)

通过日志信息我们可以知道应用程序so对应的目录为 /data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/

执行下面的命令

```shell
adb pull /data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64
kr_offline.exe demo.krhook.txt arm64
```

我们就获得了用户层的完整调用链

```shell
-----------------------------------------
openat[/storage/emulated/0/Android/data/com.DefaultCompany.krhook_unity3d/files/a.txt] dump stack:
/apex/com.android.runtime/lib64/bionic/libc.so/0x7277145000 + 0xc1388 (__openat multf3.c:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x506c18 (_ZN6il2cpp2os4File4OpenERKNSt6__ndk112basic_stringIcNS2_11char_traitsIcEENS2_9allocatorIcEEEEiiiiPi ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x528568 (_ZN6il2cpp6icalls8mscorlib6System2IO6MonoIO6Open40EPDsiiiiPi ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xf7b054 (MonoIO_Open_m194115823A6163255C8845AB97ADF010DAD88E22 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xf6e930 (MonoIO_Open_m75D574F44B3C1E6FA4E245D48D5AC73F70BE16B7 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xf6ca6c (FileStream__ctor_mBC5F76C88DBC8C81D1F83407197D75F36E1ADBD7 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xf6893c (FileStream__ctor_mB254658F1E758D76B41C942CB91BDF38FD544C83 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xf688b0 (File_Open_mDA5EB4A312EAEBF8543B13C572271FB5F673A501 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xf9b918 (FileOpenTest_FileOpen_m76B8151D8C479F745ECF6F56D62D556DB2435397 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xa8cfd8 (UnityAction_Invoke_mC9FF5AA1F82FDE635B3B6644CE71C94C31C3E71A ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xa99290 (InvokableCall_Invoke_m0B9E7F14A2C67AB51F01745BD2C6C423114C9394 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xa9c61c (UnityEvent_Invoke_mB2FA1C76256FE34D5E7F84ABE528AC61CE8A0325 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x8ea8b0 (Button_Press_m33BA6E9820146E8EED7AB489A8846D879B76CF41 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x8ea980 (Button_OnPointerClick_m4C4EDB8613C2C5B391EFD3A29C58B0AA00DD9B91 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x3ea750 (_ZN23InterfaceActionInvoker1IP58PointerEventData_tC18994283B7753E430E316A62D9E45BA6D644C63E6InvokeEjP11Il2CppClassP12Il2CppObjectS1_ ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x8d50b8 (ExecuteEvents_Execute_m24768528CCF25F4ADB0E66538ABF950C8EE2E9B0 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xbead38 (EventFunction_1_Invoke_mB923A0E7E49A56D420C97EB6D98A660EAF8A348D_gshared ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0xdc23fc (ExecuteEvents_Execute_TisRuntimeObject_m69C612263456A3111F97114B38B8A0E2E16E4347_gshared ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x3edab0 (_Z129ExecuteEvents_Execute_TisIPointerClickHandler_t337D40B4F0C87DA190B55BF225ADB716F4ADCA13_mB8A59713F468FB6A061C8A5DF7FF205EE1C9A855P52GameObject_tBD1244AD56B4E59AAD76E5E7C9282EC5CE434F0FP55BaseEventData_t46C9D2AE3183A742EDE89944AF64A23DBF1B80A5P57EventFunction_1_t7BFB6A90DB6AE5607866DE2A89133CA327285B1EPK10MethodInfo ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x8e4388 (StandaloneInputModule_ProcessTouchPress_m46FBF040EAB0A0F8D832FEB600EF0B9C48E13F61 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x8e283c (StandaloneInputModule_ProcessTouchEvents_m74C783AF0B4D517978ECCE3E8A1081F49D174F69 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x8e20cc (StandaloneInputModule_Process_mF637455BCED017FB359E090B58F15C490EFD2B54 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x34c84c (_ZN18VirtActionInvoker06InvokeEjP12Il2CppObject ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x8d37e4 (EventSystem_Update_m12CAEF521A10D406D1A6EA01E00DD851683C7208 ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x368230 (_Z65RuntimeInvoker_TrueVoid_t22962CB4C05B1D89B55A6E1139F0E87A90987017PFvvEPK10MethodInfoPvPS4_ ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x59ceac (_ZN6il2cpp2vm7Runtime6InvokeEPK10MethodInfoPvPS5_PP15Il2CppException ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libil2cpp.so/0x7184a03000 + 0x4fff04 (il2cpp_runtime_invoke ??:? )
/data/app/com.DefaultCompany.krhook_unity3d-PL6MaRBI5vxhNhVP3URXtA==/lib/arm64/libunity.so/0x7189808000 + 0xc2c1d0 (_Z23scripting_method_invoke18ScriptingMethodPtr18ScriptingObjectPtrR18ScriptingArgumentsP21ScriptingExceptionPtrb ??:? )
```

## 对应工具和链接

[https://github.com/yhnu/op7t/tree/dev/kr_offline](https://github.com/yhnu/op7t/tree/master/kr_offline)

[https://github.com/yhnu/op7t/tree/dev/krhook](https://github.com/yhnu/op7t/tree/master/krhook)

[https://github.com/yhnu/op7t/releases](https://github.com/yhnu/op7t/releases)


## 总结

通过上面的操作步骤就可以拿到对应的用户层堆栈, 为我们逆向打开一扇窗户, 后期会开发硬件断点然后离线回溯的功能, 敬请期待.











