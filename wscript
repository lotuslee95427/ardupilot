#!/usr/bin/env python3
# encoding: utf-8

# 导入未来版本的print函数
from __future__ import print_function

# 导入所需的标准库
import os.path
import os
import sys
import subprocess
import json
import fnmatch
# 将工具目录添加到Python路径
sys.path.insert(0, 'Tools/ardupilotwaf/')
sys.path.insert(0, 'Tools/scripts/')

# 导入ArduPilot WAF工具
import ardupilotwaf
import boards
import shutil
import build_options

# 从WAF库导入所需的类
from waflib import Build, ConfigSet, Configure, Context, Utils
from waflib.Configure import conf

# 为Python 2提供subprocess.run()的兼容性实现
# Ref: https://stackoverflow.com/questions/40590192/getting-an-error-attributeerror-module-object-has-no-attribute-run-while
try:
    from subprocess import CompletedProcess
except ImportError:
    # Python 2
    class CompletedProcess:
        # 构造函数,存储进程执行的参数和结果
        def __init__(self, args, returncode, stdout=None, stderr=None):
            self.args = args
            self.returncode = returncode
            self.stdout = stdout
            self.stderr = stderr

        # 检查返回码,非0则抛出异常
        def check_returncode(self):
            if self.returncode != 0:
                err = subprocess.CalledProcessError(self.returncode, self.args, output=self.stdout)
                raise err
            return self.returncode

    # 模拟Python 3的subprocess.run()函数
    def sp_run(*popenargs, **kwargs):
        input = kwargs.pop("input", None)
        check = kwargs.pop("handle", False)
        kwargs.pop("capture_output", True)
        if input is not None:
            if 'stdin' in kwargs:
                raise ValueError('stdin and input arguments may not both be used.')
            kwargs['stdin'] = subprocess.PIPE
        process = subprocess.Popen(*popenargs, **kwargs)
        try:
            outs, errs = process.communicate(input)
        except:
            process.kill()
            process.wait()
            raise
        returncode = process.poll()
        if check and returncode:
            raise subprocess.CalledProcessError(returncode, popenargs, output=outs)
        return CompletedProcess(popenargs, returncode, stdout=outs, stderr=errs)

    # 将模拟的run函数添加到subprocess模块
    subprocess.run = sp_run
    # ^ 这个monkey patch允许在Python 2和3中以相同的方式工作


# TODO: 实现'waf help'命令,显示开发者可能需要的基本任务:
# 例如如何配置板子、编译车辆、编译所有示例、添加新示例等。
# 应该适合一个终端屏幕,理想情况下命令应该可以复制粘贴。
# 添加'export waf="$PWD/waf"'技巧使其可复制粘贴。

# TODO: 用生成的ap_config.h文件替换定义
# 这至少在定义改变时使重新编译。这可能足够了。

# Linux板的默认安装前缀
default_prefix = '/usr/'

# 为自动配置目的重写Build execute和Configure post_recurse方法
Build.BuildContext.execute = ardupilotwaf.ap_autoconfigure(Build.BuildContext.execute)
Configure.ConfigurationContext.post_recurse = ardupilotwaf.ap_configure_post_recurse()

# 设置构建上下文变体
def _set_build_context_variant(board):
    for c in Context.classes:
        if not issubclass(c, Build.BuildContext):
            continue
        c.variant = board

# 删除所有子模块并同步
@conf
def submodule_force_clean(ctx):
    # 白名单目录
    whitelist = {
                            'COLCON_IGNORE',
                            'esp_idf',
                          }

    # 获取modules文件夹中的所有项目
    module_list = os.scandir('modules')

    # 删除白名单之外的所有目录
    for module in module_list:
        if (module.is_dir()) and (module.name not in whitelist):
            shutil.rmtree(module)

    submodulesync(ctx)

# 运行Tools/gittools/submodule-sync.sh同步子模块
@conf
def submodulesync(ctx):
    subprocess.call(['Tools/gittools/submodule-sync.sh'])

def init(ctx):
    # 生成任务列表,以便VS Code扩展可以跟踪构建目标的变化
    generate_tasklist(ctx, False)
    env = ConfigSet.ConfigSet()
    try:
        p = os.path.join(Context.out_dir, Build.CACHE_DIR, Build.CACHE_SUFFIX)
        env.load(p)
    except EnvironmentError:
        return

    Configure.autoconfig = 'clobber' if env.AUTOCONFIG else False

    board = ctx.options.board or env.BOARD

    if not board:
        return

    # 根据板子定义变体构建命令
    _set_build_context_variant(board)

def options(opt):
    # 加载编译器和单元测试相关选项
    opt.load('compiler_cxx compiler_c waf_unit_test python')
    opt.load('ardupilotwaf')
    opt.load('build_summary')

    g = opt.ap_groups['configure']

    # 获取可用的板子名称
    boards_names = boards.get_boards_names()
    # 获取已移除的板子名称列表
    removed_names = boards.get_removed_boards()

    # 添加--board选项,用于指定目标板子
    # action='store'表示存储选项值
    # default=None表示默认值为None
    # help参数提供选项说明,包含所有可用板子名称
    g.add_option('--board',
        action='store',
        default=None,
        help='Target board to build, choices are %s.' % ', '.join(boards_names))

    # 添加--debug选项,用于配置调试变体
    # action='store_true'表示布尔标志
    # default=False表示默认不启用调试
    g.add_option('--debug',
        action='store_true',
        default=False,
        help='Configure as debug variant.')

    # 添加--debug-symbols选项,用于添加调试符号
    # -g是该选项的短格式
    g.add_option('--debug-symbols', '-g',
        action='store_true',
        default=False,
        help='Add debug symbolds to build.')
    
    # 添加--disable-watchdog选项,用于禁用看门狗
    g.add_option('--disable-watchdog',
        action='store_true',
        default=False,
        help='Build with watchdog disabled.')

    # 添加--coverage选项,用于配置代码覆盖率标志
    g.add_option('--coverage',
                 action='store_true',
                 default=False,
                 help='Configure coverage flags.')

    # 添加--Werror选项,用于将警告视为错误
    g.add_option('--Werror',
        action='store_true',
        default=None,
        help='build with -Werror.')

    # 添加--disable-Werror选项,用于禁用将警告视为错误
    g.add_option('--disable-Werror',
        action='store_true',
        default=None,
        help='Disable -Werror.')
    
    # 添加--toolchain选项,用于覆盖默认工具链
    # 可以使用"native"来使用主机工具链
    g.add_option('--toolchain',
        action='store',
        default=None,
        help='Override default toolchain used for the board. Use "native" for using the host toolchain.')

    # 添加--disable-gccdeps选项,用于禁用GCC依赖输出方法
    g.add_option('--disable-gccdeps',
        action='store_true',
        default=False,
        help='Disable the use of GCC dependencies output method and use waf default method.')

    # 添加--enable-asserts选项,用于启用操作系统级断言
    g.add_option('--enable-asserts',
        action='store_true',
        default=False,
        help='enable OS level asserts.')

    # 添加--save-temps选项,用于保存编译器临时文件
    g.add_option('--save-temps',
        action='store_true',
        default=False,
        help='save compiler temporary files.')
    
    # 添加--enable-malloc-guard选项,用于启用内存分配保护区域
    g.add_option('--enable-malloc-guard',
        action='store_true',
        default=False,
        help='enable malloc guard regions.')

    # 添加--enable-stats选项,用于启用操作系统级线程统计
    g.add_option('--enable-stats',
        action='store_true',
        default=False,
        help='enable OS level thread statistics.')

    # 添加--bootloader选项,用于配置引导加载程序构建
    g.add_option('--bootloader',
        action='store_true',
        default=False,
        help='Configure for building a bootloader.')

    # 添加--signed-fw选项,用于配置固件签名支持
    g.add_option('--signed-fw',
        action='store_true',
        default=False,
        help='Configure for signed firmware support.')

    # 添加--private-key选项,用于指定固件签名的私钥路径
    g.add_option('--private-key',
                 action='store',
                 default=None,
            help='path to private key for signing firmware.')
    
    # 添加--no-autoconfig选项,用于禁用自动配置功能
    g.add_option('--no-autoconfig',
        dest='autoconfig',
        action='store_false',
        default=True,
        help='''Disable autoconfiguration feature. By default, the build system
triggers a reconfiguration whenever it thinks it's necessary - this
option disables that.
''')

    # 添加--no-submodule-update选项,用于禁用git子模块更新
    g.add_option('--no-submodule-update',
        dest='submodule_update',
        action='store_false',
        default=True,
        help='''Don't update git submodules. Useful for building with
submodules at specific revisions.
''')

    # 添加--enable-header-checks选项,用于启用头文件检查
    g.add_option('--enable-header-checks', action='store_true',
        default=False,
        help="Enable checking of headers")

    # 添加--default-parameters选项,用于设置要嵌入固件的默认参数
    g.add_option('--default-parameters',
        default=None,
        help='set default parameters to embed in the firmware')

    # 添加--enable-math-check-indexes选项,用于启用数学索引检查
    g.add_option('--enable-math-check-indexes',
                 action='store_true',
                 default=False,
                 help="Enable checking of math indexes")

    # 添加--disable-scripting选项,用于禁用板载脚本引擎
    g.add_option('--disable-scripting', action='store_true',
                 default=False,
                 help="Disable onboard scripting engine")

    # 添加--enable-scripting选项,用于启用板载脚本引擎
    g.add_option('--enable-scripting', action='store_true',
                 default=False,
                 help="Enable onboard scripting engine")

    # 添加--no-gcs选项,用于禁用地面站代码
    g.add_option('--no-gcs', action='store_true',
                 default=False,
                 help="Disable GCS code")
    
    # 添加--scripting-checks选项,用于启用运行时脚本完整性检查
    g.add_option('--scripting-checks', action='store_true',
                 default=True,
                 help="Enable runtime scripting sanity checks")

    # 添加--enable-onvif选项,用于启用和设置ONVIF相机控制
    g.add_option('--enable-onvif', action='store_true',
                 default=False,
                 help="Enables and sets up ONVIF camera control")

    # 添加--scripting-docs选项,用于启用脚本文档生成
    g.add_option('--scripting-docs', action='store_true',
                 default=False,
                 help="enable generation of scripting documentation")

    # 添加--enable-opendroneid选项,用于启用OpenDroneID
    g.add_option('--enable-opendroneid', action='store_true',
                 default=False,
                 help="Enables OpenDroneID")

    # 添加--enable-check-firmware选项,用于启用启动时的固件ID检查
    g.add_option('--enable-check-firmware', action='store_true',
                 default=False,
                 help="Enables firmware ID checking on boot")

    # 添加--enable-custom-controller选项,用于启用自定义控制器
    g.add_option('--enable-custom-controller', action='store_true',
                 default=False,
                 help="Enables custom controller")

    # 添加--enable-gps-logging选项,用于启用GPS日志记录
    g.add_option('--enable-gps-logging', action='store_true',
                 default=False,
                 help="Enables GPS logging")
    # 添加--enable-dds选项,用于启用DDS客户端以连接ROS2/DDS
    g.add_option('--enable-dds', action='store_true',
                 help="Enable the dds client to connect with ROS2/DDS.")

    # 添加--disable-networking选项,用于禁用网络API代码
    g.add_option('--disable-networking', action='store_true',
                 help="Disable the networking API code")

    # 添加--enable-networking-tests选项,用于启用网络测试代码,会自动启用网络功能
    g.add_option('--enable-networking-tests', action='store_true',
                 help="Enable the networking test code. Automatically enables networking.")
    
    # 添加--enable-dronecan-tests选项,用于在SITL中启用DroneCAN测试
    g.add_option('--enable-dronecan-tests', action='store_true',
                 default=False,
                 help="Enables DroneCAN tests in sitl")

    # 获取Linux选项组
    g = opt.ap_groups['linux']

    # 定义Linux相关的选项列表
    linux_options = ('--prefix', '--destdir', '--bindir', '--libdir')
    # 遍历选项列表,如果选项存在则从解析器中移除并添加到Linux选项组中
    for k in linux_options:
        option = opt.parser.get_option(k)
        if option:
            opt.parser.remove_option(k)
            g.add_option(option)

    # 添加--apstatedir选项,用于指定保存参数、日志和地形数据的位置
    g.add_option('--apstatedir',
        action='store',
        default='',
        help='''Where to save data like parameters, log and terrain.
This is the --localstatedir + ArduPilots subdirectory [default:
board-dependent, usually /var/lib/ardupilot]''')

    # 添加--rsync-dest选项,用于指定rsync命令的目标位置
    g.add_option('--rsync-dest',
        dest='rsync_dest',
        action='store',
        default='',
        help='''Destination for the rsync Waf command. It can be passed during
configuration in order to save typing.
''')

    # 添加--enable-benchmarks选项,用于启用基准测试
    g.add_option('--enable-benchmarks',
        action='store_true',
        default=False,
        help='Enable benchmarks.')

    # 添加--enable-lttng选项,用于启用lttng集成
    g.add_option('--enable-lttng', action='store_true',
        default=False,
        help="Enable lttng integration")

    # 添加--disable-libiio选项,用于禁用libiio库
    g.add_option('--disable-libiio', action='store_true',
        default=False,
        help="Don't use libiio even if supported by board and dependencies available")

    # 添加--disable-tests选项,用于禁用编译和测试执行
    g.add_option('--disable-tests', action='store_true',
        default=False,
        help="Disable compilation and test execution")

    # 添加--enable-sfml选项,用于启用SFML图形库
    g.add_option('--enable-sfml', action='store_true',
                 default=False,
                 help="Enable SFML graphics library")

    # 添加--enable-sfml-joystick选项,用于启用SFML游戏手柄输入库
    g.add_option('--enable-sfml-joystick', action='store_true',
                 default=False,
                 help="Enable SFML joystick input library")

    # 添加--enable-sfml-audio选项,用于启用SFML音频库
    g.add_option('--enable-sfml-audio', action='store_true',
                 default=False,
                 help="Enable SFML audio library")

    # 添加--osd选项,用于启用OSD(屏幕显示)支持
    g.add_option('--osd', action='store_true',
                 default=False,
                 help="Enable OSD support")

    # 添加--osd-fonts选项,用于启用带字体的OSD支持
    g.add_option('--osd-fonts', action='store_true',
                 default=False,
                 help="Enable OSD support with fonts")
    
    # 添加--sitl-osd选项,用于启用SITL的OSD功能
    g.add_option('--sitl-osd', action='store_true',
                 default=False,
                 help="Enable SITL OSD")

    # 添加--sitl-rgbled选项,用于启用SITL的RGB LED功能
    g.add_option('--sitl-rgbled', action='store_true',
                 default=False,
                 help="Enable SITL RGBLed")

    # 添加--force-32bit选项,用于强制32位构建
    g.add_option('--force-32bit', action='store_true',
                 default=False,
                 help="Force 32bit build")

    # 添加--build-dates选项,用于在二进制文件中包含构建日期
    g.add_option('--build-dates', action='store_true',
                 default=False,
                 help="Include build date in binaries.  Appears in AUTOPILOT_VERSION.os_sw_version")
    # 添加--sitl-flash-storage选项,用于启用SITL的闪存存储模拟
    # action='store_true'表示布尔标志,默认为False
    g.add_option('--sitl-flash-storage',
        action='store_true',
        default=False,
        help='Use flash storage emulation.')

    # 添加--ekf-double选项,用于将EKF(扩展卡尔曼滤波器)配置为双精度
    # action='store_true'表示布尔标志,默认为False 
    g.add_option('--ekf-double',
        action='store_true',
        default=False,
        help='Configure EKF as double precision.')

    # 添加--ekf-single选项,用于将EKF配置为单精度
    # action='store_true'表示布尔标志,默认为False
    g.add_option('--ekf-single',
        action='store_true',
        default=False,
        help='Configure EKF as single precision.')
    
    # 添加--static选项,用于强制静态构建
    # action='store_true'表示布尔标志,默认为False
    g.add_option('--static',
        action='store_true',
        default=False,
        help='Force a static build')

    # 添加--postype-single选项,用于强制postype_t类型为单精度
    # action='store_true'表示布尔标志,默认为False
    g.add_option('--postype-single',
        action='store_true',
        default=False,
        help='force single precision postype_t')

    # 添加--consistent-builds选项,用于强制构建输出一致性
    # 例如__LINE__等宏的值保持一致
    g.add_option('--consistent-builds',
        action='store_true',
        default=False,
        help='force consistent build outputs for things like __LINE__')

    # 添加--extra-hwdef选项,用于指定自定义构建的额外hwdef.dat文件
    # action='store'表示存储选项值,默认为None
    g.add_option('--extra-hwdef',
	    action='store',
	    default=None,
	    help='Extra hwdef.dat file for custom build.')

    # 添加--assert-cc-version选项,用于验证gcc编译器版本
    # 如果不是指定的gcc版本则配置失败
    g.add_option('--assert-cc-version',
                 default=None,
                 help='fail configure if not using the specified gcc version')

    # 添加--num-aux-imus选项,用于指定辅助IMU的数量
    # type='int'表示选项值为整数类型,默认为0
    g.add_option('--num-aux-imus',
                 type='int',
                 default=0,
                 help='number of auxiliary IMUs')

    # 添加--board-start-time选项,用于设置启动时的零时刻(微秒)
    # type='int'表示选项值为整数类型,默认为0
    g.add_option('--board-start-time',
                 type='int',
                 default=0,
                 help='zero time on boot in microseconds')

    # 添加--enable-new-checking选项,用于启用new操作符检查
    # 确保使用NEW_NOTHROW
    g.add_option('--enable-new-checking',
        action='store_true',
        default=False,
        help='enables checking of new to ensure NEW_NOTHROW is used')

    # 遍历build_options.py中定义的所有构建选项
    # 为每个选项添加enable和disable开关
    for opt in build_options.BUILD_OPTIONS:
        # 构造enable选项名称
        enable_option = "--" + opt.config_option()
        # 构造disable选项名称
        disable_option = enable_option.replace("--enable", "--disable")
        # 获取选项描述
        enable_description = opt.description
        # 如果描述不是以"enable"开头,则添加"Enable "前缀
        if not enable_description.lower().startswith("enable"):
            enable_description = "Enable " + enable_description
        # 构造disable选项的描述
        disable_description = "Disable " + enable_description[len("Enable "):]
        # 添加enable选项
        g.add_option(enable_option,
                     action='store_true',
                     default=False,
                     help=enable_description)
        # 添加disable选项 
        g.add_option(disable_option,
                     action='store_true',
                     default=False,
                     help=disable_description)
    # 收集自动配置文件的函数
    def _collect_autoconfig_files(cfg):
        # 遍历所有已加载的Python模块
        for m in sys.modules.values():
            paths = []
            # 如果模块有__file__属性且不为空,添加到路径列表
            if hasattr(m, '__file__') and m.__file__ is not None:
                paths.append(m.__file__)
            # 如果模块有__path__属性,将所有路径添加到列表
            elif hasattr(m, '__path__'):
                for p in m.__path__:
                    if p is not None:
                        paths.append(p)

            # 遍历所有路径
            for p in paths:
                # 如果文件已经处理过或不存在,则跳过
                if p in cfg.files or not os.path.isfile(p):
                    continue

                # 读取文件内容并更新配置的哈希值
                with open(p, 'rb') as f:
                    cfg.hash = Utils.h_list((cfg.hash, f.read()))
                    cfg.files.append(p)

    # 主配置函数
    def configure(cfg):
        # 如果没有指定开发板,默认使用SITL
        if cfg.options.board is None:
            cfg.options.board = 'sitl'

        # 获取所有支持的开发板名称
        boards_names = boards.get_boards_names()
        # 检查指定的开发板是否支持,不区分大小写
        if not cfg.options.board in boards_names:
            for b in boards_names:
                if b.upper() == cfg.options.board.upper():
                    cfg.options.board = b
                    break
        
        # 设置环境变量
        cfg.env.BOARD = cfg.options.board
        cfg.env.DEBUG = cfg.options.debug
        cfg.env.DEBUG_SYMBOLS = cfg.options.debug_symbols
        cfg.env.COVERAGE = cfg.options.coverage
        cfg.env.AUTOCONFIG = cfg.options.autoconfig

        # 设置构建上下文变体
        _set_build_context_variant(cfg.env.BOARD)
        cfg.setenv(cfg.env.BOARD)

        # 如果启用固件签名,同时启用固件检查
        if cfg.options.signed_fw:
            cfg.env.AP_SIGNED_FIRMWARE = True
            cfg.options.enable_check_firmware = True

        # 设置更多环境变量
        cfg.env.BOARD = cfg.options.board
        cfg.env.DEBUG = cfg.options.debug
        cfg.env.DEBUG_SYMBOLS = cfg.options.debug_symbols
        cfg.env.COVERAGE = cfg.options.coverage
        cfg.env.FORCE32BIT = cfg.options.force_32bit
        cfg.env.ENABLE_ASSERTS = cfg.options.enable_asserts
        cfg.env.BOOTLOADER = cfg.options.bootloader
        cfg.env.ENABLE_MALLOC_GUARD = cfg.options.enable_malloc_guard
        cfg.env.ENABLE_STATS = cfg.options.enable_stats
        cfg.env.SAVE_TEMPS = cfg.options.save_temps

        # 设置额外的硬件定义文件
        cfg.env.HWDEF_EXTRA = cfg.options.extra_hwdef
        if cfg.env.HWDEF_EXTRA:
            cfg.env.HWDEF_EXTRA = os.path.abspath(cfg.env.HWDEF_EXTRA)

        # 保存所有选项到环境变量
        cfg.env.OPTIONS = cfg.options.__dict__

        # 定义WAF_BUILD标志
        cfg.define('WAF_BUILD', 1)

        # 显示自动配置状态
        cfg.msg('Autoconfiguration', 'enabled' if cfg.options.autoconfig else 'disabled')

        # 如果启用静态链接,设置相应标志
        if cfg.options.static:
            cfg.msg('Using static linking', 'yes', color='YELLOW')
            cfg.env.STATIC_LINKING = True

        # 设置辅助IMU数量
        if cfg.options.num_aux_imus > 0:
            cfg.define('INS_AUX_INSTANCES', cfg.options.num_aux_imus)

        # 设置开发板启动时间
        if cfg.options.board_start_time != 0:
            cfg.define('AP_BOARD_START_TIME', cfg.options.board_start_time)
            cfg.env.AP_BOARD_START_TIME = cfg.options.board_start_time

        # 检查Python版本要求
        cfg.load('python')
        cfg.check_python_version(minver=(3,6,9))

        # 加载必要的WAF工具
        cfg.load('ap_library')

        # 配置选定的开发板
        cfg.msg('Setting board to', cfg.options.board)
        cfg.get_board().configure(cfg)

        # 加载其他WAF工具
        cfg.load('waf_unit_test')
        cfg.load('mavgen')
        cfg.load('dronecangen')

        # 设置子模块更新选项
        cfg.env.SUBMODULE_UPDATE = cfg.options.submodule_update

        # 检查源码是否在git仓库中
        cfg.start_msg('Source is git repository')
        if cfg.srcnode.find_node('.git'):
            cfg.end_msg('yes')
        else:
            cfg.end_msg('no')
            cfg.env.SUBMODULE_UPDATE = False

        # 显示子模块更新状态
        cfg.msg('Update submodules', 'yes' if cfg.env.SUBMODULE_UPDATE else 'no')
        cfg.load('git_submodule')

        # 加载性能测试相关工具
        if cfg.options.enable_benchmarks:
            cfg.load('gbenchmark')
        cfg.load('gtest')
        cfg.load('static_linking')
        cfg.load('build_summary')

        # 显示性能测试状态
        cfg.start_msg('Benchmarks')
        if cfg.env.HAS_GBENCHMARK:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

        # 显示单元测试状态
        cfg.start_msg('Unit tests')
        if cfg.env.HAS_GTEST:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

        # 配置脚本功能
        cfg.start_msg('Scripting')
        if cfg.options.disable_scripting:
            cfg.end_msg('disabled', color='YELLOW')
        elif cfg.options.enable_scripting:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('maybe')
        cfg.recurse('libraries/AP_Scripting')

        # 递归配置各个库
        cfg.recurse('libraries/AP_GPS')
        cfg.recurse('libraries/AP_HAL_SITL')
        cfg.recurse('libraries/SITL')
        cfg.recurse('libraries/AP_Networking')

        # 显示脚本运行时检查状态
        cfg.start_msg('Scripting runtime checks')
        if cfg.options.scripting_checks:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

        # 显示调试构建状态
        cfg.start_msg('Debug build')
        if cfg.env.DEBUG:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

        # 显示覆盖率构建状态
        cfg.start_msg('Coverage build')
        if cfg.env.COVERAGE:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

        # 显示32位构建状态
        cfg.start_msg('Force 32-bit build')
        if cfg.env.FORCE32BIT:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

        # 添加mavlink子模块
        cfg.env.append_value('GIT_SUBMODULES', 'mavlink')

        # 添加库文件包含路径
        cfg.env.prepend_value('INCLUDES', [
            cfg.srcnode.abspath() + '/libraries/',
        ])

    cfg.find_program('rsync', mandatory=False)
    if cfg.options.rsync_dest:
        cfg.msg('Setting rsync destination to', cfg.options.rsync_dest)
        cfg.env.RSYNC_DEST = cfg.options.rsync_dest

    if cfg.options.enable_header_checks:
        cfg.msg('Enabling header checks', cfg.options.enable_header_checks)
        cfg.env.ENABLE_HEADER_CHECKS = True
    else:
        cfg.env.ENABLE_HEADER_CHECKS = False

    # Always use system extensions
    cfg.define('_GNU_SOURCE', 1)

    if cfg.options.Werror:
        # print(cfg.options.Werror)
        if cfg.options.disable_Werror:
            cfg.options.Werror = False

    cfg.write_config_header(os.path.join(cfg.variant, 'ap_config.h'), guard='_AP_CONFIG_H_')

    # add in generated flags
    cfg.env.CXXFLAGS += ['-include', 'ap_config.h']

    cfg.remove_target_list()
    _collect_autoconfig_files(cfg)

def collect_dirs_to_recurse(bld, globs, **kw):
    dirs = []
    globs = Utils.to_list(globs)

    if bld.bldnode.is_child_of(bld.srcnode):
        kw['excl'] = Utils.to_list(kw.get('excl', []))
        kw['excl'].append(bld.bldnode.path_from(bld.srcnode))

    for g in globs:
        for d in bld.srcnode.ant_glob(g + '/wscript', **kw):
            dirs.append(d.parent.relpath())
    return dirs

def list_boards(ctx):
    print(*boards.get_boards_names())

def list_ap_periph_boards(ctx):
    print(*boards.get_ap_periph_boards())

@conf
def ap_periph_boards(ctx):
    return boards.get_ap_periph_boards()

vehicles = ['antennatracker', 'blimp', 'copter', 'heli', 'plane', 'rover', 'sub']

def generate_tasklist(ctx, do_print=True):
    boardlist = boards.get_boards_names()
    ap_periph_targets = boards.get_ap_periph_boards()
    tasks = []
    with open(os.path.join(Context.top_dir, "tasklist.json"), "w") as tlist:
        for board in boardlist:
            task = {}
            task['configure'] = board
            if board in ap_periph_targets:
                if 'sitl' not in board:
                    # we only support AP_Periph and bootloader builds
                    task['targets'] = ['AP_Periph', 'bootloader']
                else:
                    task['targets'] = ['AP_Periph']
            elif 'iofirmware' in board:
                task['targets'] = ['iofirmware', 'bootloader']
            else:
                if boards.is_board_based(board, boards.sitl):
                    task['targets'] = vehicles + ['replay']
                elif boards.is_board_based(board, boards.linux):
                    task['targets'] = vehicles
                else:
                    task['targets'] = vehicles + ['bootloader']
                    task['buildOptions'] = '--upload'
            tasks.append(task)
        tlist.write(json.dumps(tasks))
        if do_print:
            print(json.dumps(tasks))

def board(ctx):
    env = ConfigSet.ConfigSet()
    try:
        p = os.path.join(Context.out_dir, Build.CACHE_DIR, Build.CACHE_SUFFIX)
        env.load(p)
    except:
        print('No board currently configured')
        return

    print('Board configured to: {}'.format(env.BOARD))

def _build_cmd_tweaks(bld):
    if bld.cmd == 'check-all':
        bld.options.all_tests = True
        bld.cmd = 'check'

    if bld.cmd == 'check':
        if not bld.env.HAS_GTEST:
            bld.fatal('check: gtest library is required')
        bld.options.clear_failed_tests = True

def _build_dynamic_sources(bld):
    if not bld.env.BOOTLOADER:
        bld(
            features='mavgen',
            source='modules/mavlink/message_definitions/v1.0/all.xml',
            output_dir='libraries/GCS_MAVLink/include/mavlink/v2.0/',
            name='mavlink',
            # this below is not ideal, mavgen tool should set this, but that's not
            # currently possible
            export_includes=[
            bld.bldnode.make_node('libraries').abspath(),
            bld.bldnode.make_node('libraries/GCS_MAVLink').abspath(),
            ],
            )

    if (bld.get_board().with_can or bld.env.HAL_NUM_CAN_IFACES) and not bld.env.AP_PERIPH:
        bld(
            features='dronecangen',
            source=bld.srcnode.ant_glob('modules/DroneCAN/DSDL/[a-z]* libraries/AP_DroneCAN/dsdl/[a-z]*', dir=True, src=False),
            output_dir='modules/DroneCAN/libcanard/dsdlc_generated/',
            name='dronecan',
            export_includes=[
                bld.bldnode.make_node('modules/DroneCAN/libcanard/dsdlc_generated/include').abspath(),
                bld.srcnode.find_dir('modules/DroneCAN/libcanard/').abspath(),
                bld.srcnode.find_dir('libraries/AP_DroneCAN/canard/').abspath(),
                ]
            )
    elif bld.env.AP_PERIPH:
        bld(
            features='dronecangen',
            source=bld.srcnode.ant_glob('modules/DroneCAN/DSDL/* libraries/AP_DroneCAN/dsdl/*', dir=True, src=False),
            output_dir='modules/DroneCAN/libcanard/dsdlc_generated/',
            name='dronecan',
            export_includes=[
                bld.bldnode.make_node('modules/DroneCAN/libcanard/dsdlc_generated/include').abspath(),
                bld.srcnode.find_dir('modules/DroneCAN/libcanard/').abspath(),
            ]
        )

    if bld.env.ENABLE_DDS:
        bld.recurse("libraries/AP_DDS")

    def write_version_header(tsk):
        bld = tsk.generator.bld
        return bld.write_version_header(tsk.outputs[0].abspath())

    bld(
        name='ap_version',
        target='ap_version.h',
        vars=['AP_VERSION_ITEMS'],
        rule=write_version_header,
    )

    bld.env.prepend_value('INCLUDES', [
        bld.bldnode.abspath(),
    ])

def _build_common_taskgens(bld):
    # NOTE: Static library with vehicle set to UNKNOWN, shared by all
    # the tools and examples. This is the first step until the
    # dependency on the vehicles is reduced. Later we may consider
    # split into smaller pieces with well defined boundaries.
    bld.ap_stlib(
        name='ap',
        ap_vehicle='UNKNOWN',
        ap_libraries=bld.ap_get_all_libraries(),
    )

    if bld.env.HAS_GTEST:
        bld.libgtest(cxxflags=['-include', 'ap_config.h'])

    if bld.env.HAS_GBENCHMARK:
        bld.libbenchmark()

def _build_recursion(bld):
    common_dirs_patterns = [
        # TODO: Currently each vehicle also generate its own copy of the
        # libraries. Fix this, or at least reduce the amount of
        # vehicle-dependent libraries.
        '*',
        'Tools/*',
        'libraries/*/examples/*',
        'libraries/*/tests',
        'libraries/*/utility/tests',
        'libraries/*/benchmarks',
    ]

    common_dirs_excl = [
        'modules',
        'libraries/AP_HAL_*',
    ]

    hal_dirs_patterns = [
        'libraries/%s/tests',
        'libraries/%s/*/tests',
        'libraries/%s/*/benchmarks',
        'libraries/%s/examples/*',
    ]

    dirs_to_recurse = collect_dirs_to_recurse(
        bld,
        common_dirs_patterns,
        excl=common_dirs_excl,
    )
    if bld.env.IOMCU_FW is not None:
        if bld.env.IOMCU_FW:
            dirs_to_recurse.append('libraries/AP_IOMCU/iofirmware')

    if bld.env.PERIPH_FW is not None:
        if bld.env.PERIPH_FW:
            dirs_to_recurse.append('Tools/AP_Periph')

    dirs_to_recurse.append('libraries/AP_Scripting')

    if bld.env.ENABLE_ONVIF:
        dirs_to_recurse.append('libraries/AP_ONVIF')

    for p in hal_dirs_patterns:
        dirs_to_recurse += collect_dirs_to_recurse(
            bld,
            [p % l for l in bld.env.AP_LIBRARIES],
        )

    # NOTE: we need to sort to ensure the repeated sources get the
    # same index, and random ordering of the filesystem doesn't cause
    # recompilation.
    dirs_to_recurse.sort()

    for d in dirs_to_recurse:
        bld.recurse(d)

def _build_post_funs(bld):
    if bld.cmd == 'check':
        bld.add_post_fun(ardupilotwaf.test_summary)
    else:
        bld.build_summary_post_fun()

    if bld.env.SUBMODULE_UPDATE:
        bld.git_submodule_post_fun()

def _load_pre_build(bld):
    '''allow for a pre_build() function in build modules'''
    if bld.cmd == 'clean':
        return
    brd = bld.get_board()
    if getattr(brd, 'pre_build', None):
        brd.pre_build(bld)    

def build(bld):
    config_hash = Utils.h_file(bld.bldnode.make_node('ap_config.h').abspath())
    bld.env.CCDEPS = config_hash
    bld.env.CXXDEPS = config_hash

    bld.post_mode = Build.POST_LAZY

    bld.load('ardupilotwaf')

    bld.env.AP_LIBRARIES_OBJECTS_KW.update(
        use=['mavlink'],
        cxxflags=['-include', 'ap_config.h'],
    )

    _load_pre_build(bld)

    if bld.get_board().with_can:
        bld.env.AP_LIBRARIES_OBJECTS_KW['use'] += ['dronecan']

    _build_cmd_tweaks(bld)

    if bld.env.SUBMODULE_UPDATE:
        bld.add_group('git_submodules')
        for name in bld.env.GIT_SUBMODULES:
            bld.git_submodule(name)

    bld.add_group('dynamic_sources')
    _build_dynamic_sources(bld)

    bld.add_group('build')
    bld.get_board().build(bld)
    _build_common_taskgens(bld)

    _build_recursion(bld)

    _build_post_funs(bld)

ardupilotwaf.build_command('check',
    program_group_list='all',
    doc='builds all programs and run tests',
)
ardupilotwaf.build_command('check-all',
    program_group_list='all',
    doc='shortcut for `waf check --alltests`',
)

for name in (vehicles + ['bootloader','iofirmware','AP_Periph','replay']):
    ardupilotwaf.build_command(name,
        program_group_list=name,
        doc='builds %s programs' % name,
    )

for program_group in ('all', 'bin', 'tool', 'examples', 'tests', 'benchmarks'):
    ardupilotwaf.build_command(program_group,
        program_group_list=program_group,
        doc='builds all programs of %s group' % program_group,
    )

class LocalInstallContext(Build.InstallContext):
    """runs install using BLD/install as destdir, where BLD is the build variant directory"""
    cmd = 'localinstall'

    def __init__(self, **kw):
        super(LocalInstallContext, self).__init__(**kw)
        self.local_destdir = os.path.join(self.variant_dir, 'install')

    def execute(self):
        old_destdir = self.options.destdir
        self.options.destdir = self.local_destdir
        r = super(LocalInstallContext, self).execute()
        self.options.destdir = old_destdir
        return r

class RsyncContext(LocalInstallContext):
    """runs localinstall and then rsyncs BLD/install with the target system"""
    cmd = 'rsync'

    def __init__(self, **kw):
        super(RsyncContext, self).__init__(**kw)
        self.add_pre_fun(RsyncContext.create_rsync_taskgen)

    def create_rsync_taskgen(self):
        if 'RSYNC' not in self.env:
            self.fatal('rsync program seems not to be installed, can\'t continue')

        self.add_group()

        tg = self(
            name='rsync',
            rule='${RSYNC} -a ${RSYNC_SRC}/ ${RSYNC_DEST}',
            always=True,
        )

        tg.env.RSYNC_SRC = self.local_destdir
        if self.options.rsync_dest:
            self.env.RSYNC_DEST = self.options.rsync_dest

        if 'RSYNC_DEST' not in tg.env:
            self.fatal('Destination for rsync not defined. Either pass --rsync-dest here or during configuration.')

        tg.post()
