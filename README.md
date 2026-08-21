# Controller SDK 1.3.1（x86_64）

用法见同目录 `使用教程.md`。

- `include/controller_sdk/`  头文件
- `lib/`                     `libcontroller_sdk.so*`，可选 `libethercat.so*`
- `bin/`                     `controller_runtime` 等
- `lib/cmake/controller_sdk/`  `find_package(controller_sdk)`

只适用于 **x86_64**。另一架构请在对应机器上执行 `./scripts/package_binary_sdk.sh`。ARM 板见 `ARM编译打包.md`。

```bash
tar xzf controller-sdk-binary_1.3.1_x86_64.tar.gz
cd controller-sdk-binary_1.3.1_x86_64
export SDK_ROOT=$PWD
export LD_LIBRARY_PATH=$SDK_ROOT/lib:$LD_LIBRARY_PATH
```
