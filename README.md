# 🤖✨ rerun-robotics-cpp

把机器人状态“演”出来，不只是“算”出来。  
Rerun Viewer 里看见轨迹、误差、速度、结构变化，一眼知道系统在干什么 🔥

## 🎬 Demos

- `rerun_cpp_demo`  
  基础模型 + 动画起步款
- `rerun_showcase_demo`  
  展示增强版：关节链、目标轨迹、速度箭头、误差可视化、曲线面板

## ⚡ Quick Start

### 1) 安装rerun viewer
```bash
conda create -n rerun310 python=3.10 -y # 构建一个 python 3.10 环境
pip3 install --user rerun-sdk=0.31.3
```

### 2) 构建并安装 Arrow（静态库）

```bash
export PROJECT_ROOT=<your_project_path>/rerun_test
export ARROW_WORKDIR=<your_project_path>/arrow

mkdir -p ${PROJECT_ROOT}/third_party/arrow_install
mkdir -p ${ARROW_WORKDIR}
cd ${ARROW_WORKDIR}

git clone https://github.com/apache/arrow.git
cd arrow
mkdir -p build install

cmake -S ./cpp -B ./build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PWD/install \
  -DARROW_BUILD_STATIC=ON \
  -DARROW_BUILD_SHARED=OFF \
  -DARROW_COMPUTE=ON \
  -DARROW_CSV=OFF \
  -DARROW_DATASET=OFF \
  -DARROW_FILESYSTEM=OFF \
  -DARROW_JSON=OFF \
  -DARROW_PARQUET=OFF \
  -DARROW_WITH_BZ2=OFF \
  -DARROW_WITH_LZ4=OFF \
  -DARROW_WITH_SNAPPY=OFF \
  -DARROW_WITH_ZLIB=OFF \
  -DARROW_WITH_ZSTD=OFF \
  -DARROW_BUILD_TESTS=OFF

cmake --build ./build -j
cmake --install ./build
cp -r ./install/* ${PROJECT_ROOT}/third_party/arrow_install/
```

### 3) 编译本项目（使用本地 Arrow）

```bash
cd ${PROJECT_ROOT}
mkdir -p build
cmake -B build -S . \
  -DRERUN_DOWNLOAD_AND_BUILD_ARROW=OFF \
  -DArrow_DIR=$PWD/third_party/arrow_install/lib/cmake/Arrow
cmake --build build -j4
```

> 如果你本地已经有可用的 Arrow，也可以直接设置 `Arrow_DIR`，跳过第 1 步。

## Run

```bash
./build/rerun_cpp_demo
./build/rerun_showcase_demo
```

## 🧠 Why This Repo

- 验证 Rerun 在机器人调试场景的表达力
- 把“黑盒控制”变成“可解释画面”
- 给后续任务级可视化打底

## 🧩 Dependencies

`rerun_sdk` / `Eigen3` / `assimp` / `pinocchio` / `urdfdom`

其中 `urdfdom` 使用：
- `urdfdom_model`
- `urdfdom_world`
- `urdfdom_sensor`

## 🚀 Mesh Load Cache

为减少重复启动时的 mesh 加载耗时，`rerun_cpp_demo` 和 `rerun_showcase_demo` 都启用了本地磁盘缓存：

- 缓存目录：`/tmp/rerun_mesh_cache`
- 缓存 key：`mesh_path + scale`
- 失效策略：当原 mesh 文件修改时间晚于缓存文件时自动重建缓存

实际效果（同一模型，首次冷启动 vs 再次热启动）：
- `rerun_cpp_demo`：`~1.5s` -> `~0.1s`
- `rerun_showcase_demo`：`~1.9s` -> `~0.04s`

如果需要清空缓存重新测试冷启动：

```bash
rm -rf /tmp/rerun_mesh_cache
```

> 注意：URDF 与 mesh 路径当前为本地路径，迁移运行时需要改路径。

## 🌍 Open Source

`git@github.com:xyodfish/research_on_rerun.git`

如果你也觉得“能看见”比“能跑通”更重要，欢迎一起搞 😎
