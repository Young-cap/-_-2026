# 全局轨迹优化——2026版本
该项目是基于global_racetrajectory_optimization的翻译版本，旨在达到更好的理解，因此有作者本人观点的删改。诸位G友若有更好建议欢迎提出！

原链接：https://github.com/TUMFTM/global_racetrajectory_optimization/blob/master/Readme.md?plain=1

项目包含用于确定赛道上最佳赛车路线的算法。（赛道类似马路，赛车路线指赛车具体的行驶路径）,关于轨迹的选择有以下几个选项：

* 最短路径
* 最小曲率
* 最短时间
* 考虑动力学系统性能的最小时间（待补充：什么是考虑‘动力学系统性能’）

在弯道处，最小曲率和最短时间路线非常接近，但是一旦车辆加速度性能未能达到极限，二者就会出现差异。“最短时间”优化轨迹需要更多参数，计算时间较长。请查看main_globaltraj.py了解所有可用选项。

# 文件内容介绍
* `frictionmap`: 该文件包含生成赛道“摩擦力图”的相关函数（待确认：摩擦系数的沿赛道的分布）。
* `helper_funcs_glob`: 该文件包含一些辅助函数，用于调用（待分析：具体什么辅助函数）。
* `inputs`: 该文件夹包含车辆动力学信息，参考轨迹的csv文件和摩擦力图。
* `opt_mintime_traj`: 求解“最短时间”的函数。
  其中还包括`opt_mintime_traj/powertrain_src`中的动力学模块，该函数在考虑电池喝点状态的情况下，计算功率和热行为。（待确认）

# 轨迹规划辅助函数库
这是运行该项目不可缺少的repository，包含了轨迹规划必要的函数，需要下载。链接：https://github.com/TUMFTM/trajectory_planning_helpers.

# 环境依赖
请使用'requirements.txt'文件中指示的版本安装必要模块。（原项目代码使用Ubantu 20.04 LTS 和 python 3.7 开发）。
可能存在的安装问题：
* `cvxpy`、`cython` 或任何其他需要 `Visual C++ 编译器`的软件包：下载visual studio 2019 生成工具。（https://visualstudio.microsoft.com/de/downloads/ -> Visual Studio 2019 工具 -> 生成工具），安装它们，并选择“C++ 生成工具”选项来安装所需的 C++ 编译器及其依赖项。（作者本人未尝试过）
* quadprog 的问题：原因不明，请尝试使用 0.1.6 版本而不是 0.1.7 版本进行测试。

# 如何生成自定义的摩擦力图
 `main_gen_frictionmap.py`为'inputs'文件夹中提供的任何赛道文件创建自定义摩擦力图。生成的摩擦力图存储在'inputs/frictionmaps'文件夹中，这些摩擦力图可以用于最短时间优化。原则上，它们也可以用于最小曲率规划器的速度曲线计算。但是，我们目前不支持此功能。（待补充：赛道文件如何得到）

# 运行代码（作者使用ai，未尝试具体的调整）
* `第一步`：（可选）调整位于 `params` 文件夹中的参数文件（必需文件）。
* `第二步`：（可选）调整 `inputs/veh_dyn_info` 中的 ggv 图和 ax_max_machines 文件（如果使用）。此加速度应在不考虑阻力的情况下计算，即仅通过 F_x_drivetrain / m_veh 计算！
* `第三步`：（可选）在 `inputs/tracks` 中添加您自己的参考轨迹文件（必需文件）。
* `第四步`：（可选）在 `inputs/frictionmaps` 中添加您自己的摩擦力映射文件（如果使用）。
* `第五步`：（可选）如果您想考虑动力系统特性（热特性、功率损耗、荷电状态），请在参数文件 (`/params`) 中启用动力系统选项，并根据需要调整动力系统参数。在 `main_globaltraj.py` 文件中的字典 `imp_opts` 中设置比赛圈数，并在参数文件 (`/params`) 中（如果使用）指定非规则离散化步长，以加快优化速度。您可以通过设置 `/params/racecar.ini:simple_loss = True` 来选择动力总成组件的简单近似模型，或者通过指定 `/params/racecar.ini:simple_loss = False` 来考虑更精细的模型。
* `第六步`： 调整 `main_globaltraj.py` 文件中上方的参数并执行它以启动轨迹生成过程。
计算出的比赛轨迹存储在 `outputs/traj_race_cl.csv` 文件中。
重要提示：有关最短时间优化的更多信息，请参阅位于 `opt_mintime_traj` 文件夹中的相应 `Readme.md` 文件！
例如：柏林FE赛道的最终赛车线，opt_raceline_berlin.png

# 变量名定义一致
* path -> [x, y] 描述包含点的 x、y 坐标（即点坐标）的任何数组。\
* refline -> [x, y] 用作计算中参考线的路径。\
* reftrack -> [x, y, w_tr_right, w_tr_left] 一个数组，不仅包含参考线信息，还包含左右赛道宽度。\
在本项目中，reftrack作为用于优化的基本赛道线，包含赛道信息。（待补全：具体什么含义）
法向量通常指向行驶方向的右侧。因此，我们通过以下乘法得到轨迹边界，例如：`norm_vector * w_tr_right, -norm_vector * w_tr_left`

# 轨迹定义
目前，全局赛道轨迹优化支持两种输出格式：
* *比赛轨迹*：（默认）包含比赛轨迹的详细信息
* *LTPL 轨迹*：包含比赛轨迹的源信息以及赛道边界和参考线信息（实际的比赛轨迹需要根据存储的信息计算）。
要启用/禁用这些文件的导出，请在 `main_globtraj.py` 脚本的 `file_paths` 字典中添加相应的条目（搜索 # assemble export paths）。默认情况下，“LTPL Trajectory”的文件路径已被注释掉。

有关各个格式的详细信息，请参见下文。
### 比赛轨迹
输出的 CSV 文件包含全局比赛轨迹。数组大小为 `[no_points x 7]`，其中 no_points 取决于步长和赛道长度。结构如下：
* `s_m`：float32，米。沿赛道线的曲线距离。

* `x_m`：float32，米。赛道线点的 X 坐标。

* `y_m`：float32，米。赛道线点的 Y 坐标。

* `psi_rad`：float32，弧度。当前点赛道线的航向角，范围从 -pi 到 +pi 弧度。零点为正北（沿 y 轴）。

* `kappa_radpm`：float32，弧度/米。当前点赛道线的曲率。

* `vx_mps`：float32，米/秒。当前点的目标速度。

* `ax_mps2`：float32，米/秒²。当前点的目标加速度。我们假设该加速度从当前点到下一个点保持不变。
### LTPL 轨迹
输出的 CSV 文件包含全局比赛轨迹的源信息以及通过法向量表示的地图信息。数组大小为 `[no_points x 12]`，其中 no_points 取决于步长和赛道长度。
* x_ref_m：float32，单位为米。参考线点（例如赛道中心线）的 X 坐标。

* y_ref_m：float32，单位为米。参考线点（例如赛道中心线）的 Y 坐标。

* width_right_m：float32，单位为米。参考线点到赛道右侧边界（沿法向量方向）的距离。

* width_left_m：float32，单位为米。参考线点到赛道左侧边界（沿法向量方向）的距离。

* x_normvec_m：float32，单位为米。基于参考线点的归一化法向量的 X 坐标。

* y_normvec_m：float32，单位为米。基于参考线点的归一化法向量的 Y 坐标。

* alpha_m：float32，米。保留参考线点横向位移（单位：米）的优化问题的解。

* s_racetraj_m：float32，米。沿赛道的曲线距离。

* psi_racetraj_rad：float32，弧度。当前点赛道的航向角，范围从 -pi 到 +pi 弧度。零点为正北。

* kappa_racetraj_radpm：float32，弧度/米。当前点赛道的曲率。

* vx_racetraj_mps：float32，米/秒。当前点的目标速度。

* ax_racetraj_mps2：float32，米/秒²。当前点的目标加速度。假设该加速度从当前点到下一个点保持不变。
生成的文件可以直接导入到[基于图的局部轨迹规划器](https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner)中。(这是做什么用的？)

# 参考
* 最小曲率轨迹规划\Heilmeier、Wischnewski、Hermansdorfer、Betz、Lienkamp、Lohmann\
自动赛车的最小曲率轨迹规划与控制\DOI: 10.1080/00423114.2019.1631455\联系人：[Alexander Heilmeier](mailto:alexander.heilmeier@tum.de)。

* 时间最优轨迹规划\Christ、Wischnewski、Heilmeier、Lohmann\考虑可变轮胎-路面摩擦系数的赛车时间最优轨迹规划\DOI：10.1080/00423114.2019.1704804\联系人：Fabian Christ（邮箱：fabian.christ@tum.de）

* 摩擦力图生成\Hermansdorfer、Betz、Lienkamp\用于自动驾驶赛车轮胎-路面摩擦力潜力估计和预测的概念\DOI：10.1109/ITSC.2019.8917024\联系人：Leonhard Hermansdorfer（邮箱：leo.hermansdorfer@tum.de）

* 动力系统行为\Herrmann、Passigato、Betz、Lienkamp\自动驾驶电动赛车的最短比赛时间规划策略\DOI：10.1109/ITSC45102.2020.9294681\预印本：https://arxiv.org/abs/2005.07127 \联系人：[Thomas Herrmann](mailto:thomas.herrmann@tum.de)。

2026.6.2
