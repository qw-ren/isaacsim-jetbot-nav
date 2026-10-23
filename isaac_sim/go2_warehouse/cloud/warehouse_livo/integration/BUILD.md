# 隔离构建来源

- ROS2移植：https://github.com/Robotic-Developer-Road/FAST-LIVO2，commit见SOURCE_PINS.json；不是官方ROS2版本。
- Vikit：https://github.com/Robotic-Developer-Road/rpg_vikit，commit见SOURCE_PINS.json。
- Sophus：https://github.com/strasdat/Sophus，tag1.22.10对应固定commit见SOURCE_PINS.json。

实际试验在独立build_source中放fast_livo、vikit，以及Sophus的sophus头文件目录；根目录使用本CMakeLists.txt。integration.patch以该根目录为基准，patch -p1应用到固定源码。其修改为ImageTransport有效节点、显式本地相机参数、点时间秒转毫秒、实际状态测量时间和额外输出视觉更新后状态；不改估计数学更新。

source /opt/ros/humble/setup.bash 后，在单独build目录CMake配置并构建fastlivo_mapping。保留已有ROS/PCL/OpenCV及依赖；本地构建曾需显式指定系统fmt_DIR和CURL_LIBRARY_RELEASE，排除Conda库混用。此文档记录本次方法，不是所有机器通用安装器，不要求更换Isaac/CUDA/PyTorch。CMake中的运行输出根目录指向本次云端私有runtime目录，迁移时应显式确认输出位置及权限。

本次二进制SHA见PROVENANCE.json。二进制及私有动态库包不入Git；云端ldd和文件SHA已核验，本地另存构建日志/完整源码/运行包。GPL许可证按固定上游保留UPSTREAM_LICENSE，不把package.xml旧BSD字段视为实际许可证。Vikit、Sophus保留各自源码的原始许可证。
