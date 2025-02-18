#!/usr/bin/env python3
"""
ArduPilot自动测试套件。

Andrew Tridgell, October 2011

 AP_FLAKE8_CLEAN
"""
from __future__ import print_function
import atexit  # 用于注册退出时的清理函数
import fnmatch  # 用于文件名匹配
import copy  # 用于对象的深拷贝
import glob  # 用于文件路径匹配
import optparse  # 用于命令行参数解析
import os  # 用于操作系统相关功能
import re  # 用于正则表达式
import shutil  # 用于文件操作
import signal  # 用于信号处理
import subprocess  # 用于执行子进程
import sys  # 用于系统相关功能
import time  # 用于时间相关功能
import traceback  # 用于异常跟踪

# 导入各个飞行器类型的测试模块
import blimp
import rover
import arducopter
import arduplane
import ardusub
import antennatracker
import quadplane
import balancebot
import sailboat
import helicopter

import examples
from pysim import util
from pymavlink.generator import mavtemplate

from vehicle_test_suite import Test

tester = None

build_opts = None


def buildlogs_dirpath():
    """返回BUILDLOGS目录路径。"""
    return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))


def buildlogs_path(path):
    """返回buildlogs目录中的文件路径。"""
    bits = [buildlogs_dirpath()]
    if isinstance(path, list):
        bits.extend(path)
    else:
        bits.append(path)
    return os.path.join(*bits)


def build_all_filepath():
    """获取build_all.sh脚本路径。"""
    return util.reltopdir('Tools/scripts/build_all.sh')


def build_all():
    """运行build_all.sh脚本。"""
    print("Running build_all.sh")
    if util.run_cmd(build_all_filepath(), directory=util.reltopdir('.')) != 0:
        print("Failed build_all.sh")
        return False
    return True


def build_binaries():
    """运行build_binaries.py脚本。"""
    print("Running build_binaries.py")

    # 复制脚本(和脚本使用的各种库)因为它会切换git分支,这可能会在运行时改变脚本
    for thing in [
            "board_list.py",
            "build_binaries_history.py", 
            "build_binaries.py",
            "build_sizes/build_sizes.py",
            "generate_manifest.py",
            "gen_stable.py",
    ]:
        orig = util.reltopdir('Tools/scripts/%s' % thing)
        copy = util.reltopdir('./%s' % os.path.basename(thing))
        shutil.copy2(orig, copy)

    if util.run_cmd("./build_binaries.py", directory=util.reltopdir('.')) != 0:
        print("Failed build_binaries.py")
        return False
    return True


def build_examples(**kwargs):
    """编译示例程序。"""
    for target in 'Pixhawk1', 'navio', 'linux':
        print("Running build.examples for %s" % target)
        try:
            util.build_examples(target, **kwargs)
        except Exception as e:
            print("Failed build_examples on board=%s" % target)
            print(str(e))
            return False

    return True


def build_unit_tests(**kwargs):
    """编译单元测试。"""
    for target in ['linux', 'sitl']:
        print("Running build.unit_tests for %s" % target)
        try:
            util.build_tests(target, **kwargs)
        except Exception as e:
            print("Failed build.unit_tests on board=%s" % target)
            print(str(e))
            return False

    return True


def run_unit_test(test):
    """运行单个单元测试文件。"""
    print("Running (%s)" % test)
    subprocess.check_call([test])


def run_unit_tests():
    """运行所有单元测试文件。"""
    success = True
    fail_list = []
    for target in ['linux', 'sitl']:
        binary_dir = util.reltopdir(os.path.join('build',
                                                 target,
                                                 'tests',
                                                 ))
        tests = glob.glob("%s/*" % binary_dir)
        for test in tests:
            try:
                run_unit_test(test)
            except subprocess.CalledProcessError:
                print("Exception running (%s)" % test)
                fail_list.append(target + '/' + os.path.basename(test))
                success = False

    print("Failing tests:")
    for failure in fail_list:
        print("  %s" % failure)
    return success


def run_clang_scan_build():
    """运行Clang Scan-build工具。"""
    if util.run_cmd("scan-build python waf configure",
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-configure")
        return False

    if util.run_cmd("scan-build python waf clean",
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-clean")
        return False

    if util.run_cmd("scan-build python waf build",
                    directory=util.reltopdir('.')) != 0:
        print("Failed scan-build-build")
        return False

    return True


def param_parse_filepath():
    """获取param_parse.py脚本路径。"""
    return util.reltopdir('Tools/autotest/param_metadata/param_parse.py')


def all_vehicles():
    """获取所有飞行器名称。"""
    return ('ArduPlane',
            'ArduCopter',
            'Rover',
            'AntennaTracker',
            'ArduSub',
            'Blimp',
            'AP_Periph',
            )


def build_parameters():
    """运行param_parse.py脚本。"""
    print("Running param_parse.py")
    for vehicle in all_vehicles():
        if util.run_cmd([param_parse_filepath(), '--vehicle', vehicle],
                        directory=util.reltopdir('.')) != 0:
            print("Failed param_parse.py (%s)" % vehicle)
            return False
    return True


def mavtogpx_filepath():
    """获取mavtogpx脚本路径。"""
    return util.reltopdir("modules/mavlink/pymavlink/tools/mavtogpx.py")


def convert_gpx():
    """将tlog文件转换为GPX和KML格式。"""
    mavlog = glob.glob(buildlogs_path("*.tlog"))
    passed = True
    for m in mavlog:
        util.run_cmd(mavtogpx_filepath() + " --nofixcheck " + m)
        gpx = m + '.gpx'
        kml = m + '.kml'
        try:
            util.run_cmd('gpsbabel -i gpx -f %s '
                         '-o kml,units=m,floating=1,extrude=1 -F %s' %
                         (gpx, kml))
        except subprocess.CalledProcessError:
            passed = False
        try:
            util.run_cmd('zip %s.kmz %s.kml' % (m, m))
        except subprocess.CalledProcessError:
            passed = False
        util.run_cmd("mavflightview.py --imagefile=%s.png %s" % (m, m))
    return passed


def test_prerequisites():
    """检查是否有正确的目录和工具来运行测试。"""
    print("Testing prerequisites")
    util.mkdir_p(buildlogs_dirpath())
    return True


def alarm_handler(signum, frame):
    """处理测试超时。"""
    global results, opts, tester
    try:
        print("Alarm handler called")
        if tester is not None:
            if tester.rc_thread is not None:
                tester.rc_thread_should_quit = True
                tester.rc_thread.join()
                tester.rc_thread = None
        results.add('TIMEOUT',
                    '<span class="failed-text">FAILED</span>',
                    opts.timeout)
        util.pexpect_close_all()
        convert_gpx()
        write_fullresults()
        os.killpg(0, signal.SIGKILL)
    except Exception:
        pass
    sys.exit(1)


def should_run_step(step):
    """判断是否应该跳过某个步骤。"""
    for skip in skipsteps:
        if fnmatch.fnmatch(step.lower(), skip.lower()):
            return False
    return True


__bin_names = {
    "Copter": "arducopter",
    "CopterTests1a": "arducopter",
    "CopterTests1b": "arducopter",
    "CopterTests1c": "arducopter",
    "CopterTests1d": "arducopter",
    "CopterTests1e": "arducopter",

    "CopterTests2a": "arducopter",
    "CopterTests2b": "arducopter",

    "Plane": "arduplane",
    "Rover": "ardurover",
    "Tracker": "antennatracker",
    "Helicopter": "arducopter-heli",
    "QuadPlane": "arduplane",
    "Sub": "ardusub",
    "Blimp": "blimp",
    "BalanceBot": "ardurover",
    "Sailboat": "ardurover",
    "SITLPeriphUniversal": ("sitl_periph_universal", "AP_Periph"),
    "SITLPeriphBattMon": ("sitl_periph_battmon", "AP_Periph"),
    "CAN": "arducopter",
    "BattCAN": "arducopter",
}


def binary_path(step, debug=False):
    """获取飞行器二进制文件路径。"""
    try:
        vehicle = step.split(".")[1]
    except Exception:
        return None

    if vehicle not in __bin_names:
        # 处理没有特定二进制文件的构建
        return None

    try:
        (config_name, binary_name) = __bin_names[vehicle]
    except ValueError:
        config_name = "sitl"
        binary_name = __bin_names[vehicle]

    binary = util.reltopdir(os.path.join('build',
                                         config_name,
                                         'bin',
                                         binary_name))
    if not os.path.exists(binary):
        if os.path.exists(binary + ".exe"):
            binary += ".exe"
        else:
            raise ValueError("Binary (%s) does not exist" % (binary,))

    return binary


def split_specific_test_step(step):
    """从参数中提取测试。"""
    print('step=%s' % str(step))
    m = re.match("((fly|drive|dive|test)[.][^.]+)[.](.*)", step)
    if m is None:
        return None
    return ((m.group(1), m.group(3)))


def find_specific_test_to_run(step):
    """在参数中查找要运行的测试。"""
    t = split_specific_test_step(step)
    if t is None:
        return None
    (testname, test) = t
    return "%s.%s" % (testname, test)


tester_class_map = {
    "test.Blimp": blimp.AutoTestBlimp,
    "test.Copter": arducopter.AutoTestCopter,
    "test.CopterTests1a": arducopter.AutoTestCopterTests1a, # 8m43s
    "test.CopterTests1b": arducopter.AutoTestCopterTests1b, # 8m5s
    "test.CopterTests1c": arducopter.AutoTestCopterTests1c, # 5m17s
    "test.CopterTests1d": arducopter.AutoTestCopterTests1d, # 8m20s
    "test.CopterTests1e": arducopter.AutoTestCopterTests1e, # 8m32s
    "test.CopterTests2a": arducopter.AutoTestCopterTests2a, # 8m23s
    "test.CopterTests2b": arducopter.AutoTestCopterTests2b, # 8m18s
    "test.Plane": arduplane.AutoTestPlane,
    "test.QuadPlane": quadplane.AutoTestQuadPlane,
    "test.Rover": rover.AutoTestRover,
    "test.BalanceBot": balancebot.AutoTestBalanceBot,
    "test.Sailboat": sailboat.AutoTestSailboat,
    "test.Helicopter": helicopter.AutoTestHelicopter,
    "test.Sub": ardusub.AutoTestSub,
    "test.Tracker": antennatracker.AutoTestTracker,
    "test.CAN": arducopter.AutoTestCAN,
    "test.BattCAN": arducopter.AutoTestBattCAN,
}

supplementary_test_binary_map = {
    "test.CAN": ["sitl_periph_universal:AP_Periph:0:Tools/autotest/default_params/periph.parm,Tools/autotest/default_params/quad-periph.parm", # noqa: E501
                 "sitl_periph_universal:AP_Periph:1:Tools/autotest/default_params/periph.parm"],
    "test.BattCAN": [
        "sitl_periph_battmon:AP_Periph:0:Tools/autotest/default_params/periph-battmon.parm,Tools/autotest/default_params/quad-periph.parm", # noqa: E501
    ],
}


def run_specific_test(step, *args, **kwargs):
    """运行特定测试。"""
    t = split_specific_test_step(step)
    if t is None:
        return []
    (testname, test) = t

    tester_class = tester_class_map[testname]
    global tester
    tester = tester_class(*args, **kwargs)

    # print("Got %s" % str(tester))
    for a in tester.tests():
        if not isinstance(a, Test):
            a = Test(a)
        print("Got %s" % (a.name))
        if a.name == test:
            return tester.autotest(tests=[a], allow_skips=False, step_name=step), tester
    print("Failed to find test %s on %s" % (test, testname))
    sys.exit(1)


def run_step(step):
    """运行一个步骤。"""
    # 删除旧日志
    util.run_cmd('/bin/rm -f logs/*.BIN logs/LASTLOG.TXT')

    if step == "prerequisites":
        return test_prerequisites()

    build_opts = {
        "j": opts.j,
        "debug": opts.debug,
        "clean": not opts.no_clean,
        "configure": not opts.no_configure,
        "math_check_indexes": opts.math_check_indexes,
        "ekf_single": opts.ekf_single,
        "postype_single": opts.postype_single,
        "extra_configure_args": opts.waf_configure_args,
        "coverage": opts.coverage,
        "force_32bit" : opts.force_32bit,
        "ubsan" : opts.ubsan,
        "ubsan_abort" : opts.ubsan_abort,
        "num_aux_imus" : opts.num_aux_imus,
        "dronecan_tests" : opts.dronecan_tests,
    }

    if opts.Werror:
        build_opts['extra_configure_args'].append("--Werror")

    build_opts = build_opts

    vehicle_binary = None
    board = "sitl"
    if step == 'build.Plane':
        vehicle_binary = 'bin/arduplane'

    if step == 'build.Rover':
        vehicle_binary = 'bin/ardurover'

    if step == 'build.Copter':
        vehicle_binary = 'bin/arducopter'

    if step == 'build.Blimp':
        vehicle_binary = 'bin/blimp'

    if step == 'build.Tracker':
        vehicle_binary = 'bin/antennatracker'

    if step == 'build.Helicopter':
        vehicle_binary = 'bin/arducopter-heli'

    if step == 'build.Sub':
        vehicle_binary = 'bin/ardusub'

    if step == 'build.SITLPeriphUniversal':
        vehicle_binary = 'bin/AP_Periph'
        board = 'sitl_periph_universal'

    if step == 'build.SITLPeriphBattMon':
        vehicle_binary = 'bin/AP_Periph'
        board = 'sitl_periph_battmon'

    if step == 'build.Replay':
        return util.build_replay(board='SITL')

    if vehicle_binary is not None:
        try:
            binary = binary_path(step, debug=opts.debug)
            os.unlink(binary)
        except (FileNotFoundError, ValueError):
            pass
        return util.build_SITL(
            vehicle_binary,
            board=board,
            **build_opts
        )

    binary = binary_path(step, debug=opts.debug)

    # 检查是否需要任何补充二进制文件
    supplementary_binaries = []
    for k in supplementary_test_binary_map.keys():
        if step.startswith(k):
            # 此测试需要使用补充二进制文件
            for supplementary_test_binary in supplementary_test_binary_map[k]:
                a = supplementary_test_binary.split(':')
                if len(a) != 4:
                    raise ValueError("Bad supplementary_test_binary %s" % supplementary_test_binary)
                config_name = a[0]
                binary_name = a[1]
                instance_num = int(a[2])
                param_file = a[3].split(",")
                bin_path = util.reltopdir(os.path.join('build', config_name, 'bin', binary_name))
                customisation = '-I {}'.format(instance_num)
                sup_binary = {"binary" : bin_path,
                              "customisation" : customisation,
                              "param_file" : param_file}
                supplementary_binaries.append(sup_binary)
            # 我们正在与补充应用程序一起运行
            # 不能加速
            opts.speedup = 1.0
            break

    fly_opts = {
        "viewerip": opts.viewerip,
        "use_map": opts.map,
        "valgrind": opts.valgrind,
        "callgrind": opts.callgrind,
        "gdb": opts.gdb,
        "gdb_no_tui": opts.gdb_no_tui,
        "lldb": opts.lldb,
        "gdbserver": opts.gdbserver,
        "breakpoints": opts.breakpoint,
        "disable_breakpoints": opts.disable_breakpoints,
        "_show_test_timings": opts.show_test_timings,
        "force_ahrs_type": opts.force_ahrs_type,
        "num_aux_imus" : opts.num_aux_imus,
        "replay": opts.replay,
        "logs_dir": buildlogs_dirpath(),
        "sup_binaries": supplementary_binaries,
        "reset_after_every_test": opts.reset_after_every_test,
        "build_opts": copy.copy(build_opts),
        "generate_junit": opts.junit,
        "enable_fgview": opts.enable_fgview,
    }
    if opts.speedup is not None:
        fly_opts["speedup"] = opts.speedup

    # 处理"test.Copter"等:
    if step in tester_class_map:
        # 创建测试类的实例:
        global tester
        tester = tester_class_map[step](binary, **fly_opts)
        # 运行测试并返回其结果和测试器本身
        return tester.autotest(None, step_name=step), tester

    # 处理"test.Copter.CPUFailsafe"等:
    specific_test_to_run = find_specific_test_to_run(step)
    if specific_test_to_run is not None:
        return run_specific_test(specific_test_to_run, binary, **fly_opts)

    if step == 'build.All':
        return build_all()

    if step == 'build.Binaries':
        return build_binaries()

    if step == 'build.examples':
        return build_examples(**build_opts)

    if step == 'run.examples':
        return examples.run_examples(debug=opts.debug, valgrind=False, gdb=False)

    if step == 'build.Parameters':
        return build_parameters()

    if step == 'convertgpx':
        return convert_gpx()

    if step == 'build.unit_tests':
        return build_unit_tests(**build_opts)

    if step == 'run.unit_tests':
        return run_unit_tests()

    if step == 'clang-scan-build':
        return run_clang_scan_build()

    raise RuntimeError("Unknown step %s" % step)


class TestResult(object):
    """测试结果类。"""

    def __init__(self, name, result, elapsed):
        """初始化测试结果类。"""
        self.name = name
        self.result = result
        self.elapsed = "%.1f" % elapsed


class TestFile(object):
    """测试结果文件。"""

    def __init__(self, name, fname):
        """初始化测试结果文件。"""
        self.name = name
        self.fname = fname


class TestResults(object):
    """测试结果类。"""

    def __init__(self):
        """初始化测试结果类。"""
        self.date = time.asctime()
        self.githash = util.get_git_hash()
        self.tests = []
        self.files = []
        self.images = []

    def add(self, name, result, elapsed):
        """添加一个结果。"""
        self.tests.append(TestResult(name, result, elapsed))

    def addfile(self, name, fname):
        """添加一个结果文件。"""
        self.files.append(TestFile(name, fname))

    def addimage(self, name, fname):
        """添加一个结果图像。"""
        self.images.append(TestFile(name, fname))

    def addglob(self, name, pattern):
        """添加一组文件。"""
        for f in glob.glob(buildlogs_path(pattern)):
            self.addfile(name, os.path.basename(f))

    def addglobimage(self, name, pattern):
        """添加一组图像。"""
        for f in glob.glob(buildlogs_path(pattern)):
            self.addimage(name, os.path.basename(f))

    def generate_badge(self):
        """获取徽章模板,填充并将结果保存到buildlogs路径。"""
        passed_tests = len([t for t in self.tests if "PASSED" in t.result])
        total_tests = len(self.tests)
        badge_color = "#4c1" if passed_tests == total_tests else "#e05d44"

        badge_text = "{0}/{1}".format(passed_tests, total_tests)
        # 文本长度,使其不被svg拉伸
        text_length = len(badge_text) * 70

        # 加载模板文件
        template_path = 'Tools/autotest/web/autotest-badge-template.svg'
        with open(util.reltopdir(template_path), "r") as f:
            template = f.read()

        # 将结果添加到模板中
        badge = template.format(color=badge_color,
                                text=badge_text,
                                text_length=text_length)
        with open(buildlogs_path("autotest-badge.svg"), "w") as f:
            f.write(badge)


def copy_tree(f, t, dirs_exist_ok=False):
    shutil.copytree(f, t, dirs_exist_ok=dirs_exist_ok)


def write_webresults(results_to_write):
    """写入网页结果。"""
    t = mavtemplate.MAVTemplate()
    for h in glob.glob(util.reltopdir('Tools/autotest/web/*.html')):
        html = util.loadfile(h)
        f = open(buildlogs_path(os.path.basename(h)), mode='w')
        t.write(f, html, results_to_write)
        f.close()
    for f in glob.glob(util.reltopdir('Tools/autotest/web/*.png')):
        shutil.copy(f, buildlogs_path(os.path.basename(f)))
    copy_tree(util.reltopdir("Tools/autotest/web/css"), buildlogs_path("css"), dirs_exist_ok=True)
    results_to_write.generate_badge()


def write_fullresults():
    """写出完整的结果集。"""
    global results
    results.addglob("Google Earth track", '*.kmz')
    results.addfile('Full Logs', 'autotest-output.txt')
    results.addglob('DataFlash Log', '*-log.bin')
    results.addglob("MAVLink log", '*.tlog')
    results.addglob("GPX track", '*.gpx')

    # 所有飞行器通用的结果:
    vehicle_files = [
        ('{vehicle} core', '{vehicle}.core'),
        ('{vehicle} ELF', '{vehicle}.elf'),
    ]
    vehicle_globs = [('{vehicle} log', '{vehicle}-*.BIN'), ]
    for vehicle in all_vehicles():
        subs = {'vehicle': vehicle}
        for vehicle_file in vehicle_files:
            description = vehicle_file[0].format(**subs)
            filename = vehicle_file[1].format(**subs)
            results.addfile(description, filename)
        for vehicle_glob in vehicle_globs:
            description = vehicle_glob[0].format(**subs)
            glob = vehicle_glob[1].format(**subs)
            results.addglob(description, glob)

    results.addglob("CopterAVC log", 'CopterAVC-*.BIN')
    results.addfile("CopterAVC core", 'CopterAVC.core')

    results.addglob('APM:Libraries documentation', 'docs/libraries/index.html')
    results.addglob('APM:Plane documentation', 'docs/ArduPlane/index.html')
    results.addglob('APM:Copter documentation', 'docs/ArduCopter/index.html')
    results.addglob('APM:Rover documentation', 'docs/Rover/index.html')
    results.addglob('APM:Sub documentation', 'docs/ArduSub/index.html')
    results.addglob('APM:Blimp documentation', 'docs/Blimp/index.html')
    results.addglobimage("Flight Track", '*.png')

    write_webresults(results)


def run_tests(steps):
    """运行一系列步骤。"""
    global results

    corefiles = glob.glob("core*")
    corefiles.extend(glob.glob("ap-*.core"))
    if corefiles:
        print('Removing corefiles: %s' % str(corefiles))
        for f in corefiles:
            os.unlink(f)

    diagnostic_files = []
    for p in "dumpstack.sh_*", "dumpcore.sh_*", "autotest-*tlog":
        diagnostic_files.extend(glob.glob(p))
    if diagnostic_files:
        print('Removing diagnostic files: %s' % str(diagnostic_files))
        for f in diagnostic_files:
            os.unlink(f)

    passed = True
    failed = []
    failed_testinstances = dict()
    for step in steps:
        util.pexpect_close_all()

        t1 = time.time()
        print(">>>> RUNNING STEP: %s at %s" % (step, time.asctime()))
        try:
            success = run_step(step)
            testinstance = None
            if isinstance(success, tuple):
                (success, testinstance) = success
            if success:
                results.add(step, '<span class="passed-text">PASSED</span>',
                            time.time() - t1)
                print(">>>> PASSED STEP: %s at %s" % (step, time.asctime()))
            else:
                print(">>>> FAILED STEP: %s at %s" % (step, time.asctime()))
                passed = False
                failed.append(step)
                if testinstance is not None:
                    if failed_testinstances.get(step) is None:
                        failed_testinstances[step] = []
                    failed_testinstances[step].append(testinstance)
                results.add(step, '<span class="failed-text">FAILED</span>',
                            time.time() - t1)
        except Exception as msg:
            passed = False
            failed.append(step)
            print(">>>> FAILED STEP: %s at %s (%s)" %
                  (step, time.asctime(), msg))
            traceback.print_exc(file=sys.stdout)
            results.add(step,
                        '<span class="failed-text">FAILED</span>',
                        time.time() - t1)

        global tester
        if tester is not None and tester.rc_thread is not None:
            if passed:
                print("BAD: RC Thread still alive after run_step")
            tester.rc_thread_should_quit = True
            tester.rc_thread.join()
            tester.rc_thread = None

    if not passed:
        keys = failed_testinstances.keys()
        if len(keys):
            print("Failure Summary:")
        for key in keys:
            print("  %s:" % key)
            for testinstance in failed_testinstances[key]:
                for failure in testinstance.fail_list:
                    print("  " + str(failure))

        print("FAILED %u tests: %s" % (len(failed), failed))

    util.pexpect_close_all()

    write_fullresults()

    return passed


vehicle_list = ['Sub', 'Copter', 'Plane', 'Tracker', 'Rover', 'QuadPlane', 'BalanceBot', 'Helicopter', 'Sailboat', 'Blimp']


def list_subtests():
    """Print the list of tests and tests description for each vehicle."""
    for vehicle in sorted(vehicle_list):
        tester_class = tester_class_map["test.%s" % vehicle]
        tester = tester_class("/bin/true", None)
        subtests = tester.tests()
        sorted_list = []
        for subtest in subtests:
            if str(type(subtest)) == "<class 'method'>":
                subtest = Test(subtest)
            sorted_list.append([subtest.name, subtest.description])
        sorted_list.sort()

        print("%s:" % vehicle)
        for subtest in sorted_list:
            print("    %s: %s" % (subtest[0], subtest[1]))
        print("")


def list_subtests_for_vehicle(vehicle_type):
    """Print the list of tests for a vehicle."""
    # Check that we aren't in a sub test
    if "Test" in vehicle_type:
        vehicle_type = re.findall('[A-Z][a-z0-9]*', vehicle_type)[0]
    if vehicle_type in vehicle_list:
        tester_class = tester_class_map["test.%s" % vehicle_type]
        tester = tester_class("/bin/true", None)
        subtests = tester.tests()
        sorted_list = []
        for subtest in subtests:
            if not isinstance(subtest, Test):
                subtest = Test(subtest)
            sorted_list.append([subtest.name, subtest.description])
        sorted_list.sort()
        for subtest in sorted_list:
            print("%s " % subtest[0], end='')
        print("")  # needed to clear the trailing %


if __name__ == "__main__":
    ''' main program '''
    os.environ['PYTHONUNBUFFERED'] = '1'

    if sys.platform != "darwin":
        os.putenv('TMPDIR', util.reltopdir('tmp'))

    class MyOptionParser(optparse.OptionParser):
        """Custom option parse class."""

        def format_epilog(self, formatter):
            """Retun customized option parser epilog."""
            return self.epilog

    parser = MyOptionParser(
        "autotest", epilog=""
        "e.g. autotest.py build.Rover test.Rover # test Rover\n"
        "e.g. autotest.py build.Rover test.Rover build.Plane test.Plane # test Rover and Plane\n"
        "e.g. autotest.py --debug --valgrind build.Rover test.Rover # test Rover under Valgrind\n"
        "e.g. autotest.py --debug --gdb build.Tracker test.Tracker # run Tracker under gdb\n"
        "e.g. autotest.py --debug --gdb build.Sub test.Sub.DiveManual # do specific Sub test\n"
    )
    parser.add_option("--autotest-server",
                      action='store_true',
                      default=False,
                      help='Run in autotest-server mode; dangerous!')
    parser.add_option("--skip",
                      type='string',
                      default='',
                      help='list of steps to skip (comma separated)')
    parser.add_option("--list",
                      action='store_true',
                      default=False,
                      help='list the available steps')
    parser.add_option("--list-subtests",
                      action='store_true',
                      default=False,
                      help='list available subtests e.g. test.Copter')
    parser.add_option("--viewerip",
                      default=None,
                      help='IP address to send MAVLink and fg packets to')
    parser.add_option("--enable-fgview",
                      action='store_true',
                      help="Enable FlightGear output")
    parser.add_option("--map",
                      action='store_true',
                      default=False,
                      help='show map')
    parser.add_option("--experimental",
                      default=False,
                      action='store_true',
                      help='enable experimental tests')
    parser.add_option("--timeout",
                      default=None,
                      type='int',
                      help='maximum runtime in seconds')
    parser.add_option("--show-test-timings",
                      action="store_true",
                      default=False,
                      help="show how long each test took to run")
    parser.add_option("--validate-parameters",
                      action="store_true",
                      default=False,
                      help="validate vehicle parameter files")
    parser.add_option("--Werror",
                      action='store_true',
                      default=False,
                      help='configure with --Werror')
    parser.add_option("--junit",
                      default=False,
                      action='store_true',
                      help='Generate Junit XML tests report')

    group_build = optparse.OptionGroup(parser, "Build options")
    group_build.add_option("--no-configure",
                           default=False,
                           action='store_true',
                           help='do not configure before building',
                           dest="no_configure")
    group_build.add_option("", "--waf-configure-args",
                           action="append",
                           dest="waf_configure_args",
                           type="string",
                           default=[],
                           help="extra arguments passed to waf in configure")
    group_build.add_option("-j", default=None, type='int', help='build CPUs')
    group_build.add_option("--no-clean",
                           default=False,
                           action='store_true',
                           help='do not clean before building',
                           dest="no_clean")
    group_build.add_option("--debug",
                           default=None,
                           action='store_true',
                           help='make built SITL binaries debug binaries')
    group_build.add_option("--no-debug",
                           default=None,
                           action='store_true',
                           help='do not make built SITL binaries debug binaries')
    group_build.add_option("--coverage",
                           default=False,
                           action='store_true',
                           help='make built binaries coverage binaries')
    group_build.add_option("--enable-math-check-indexes",
                           default=False,
                           action="store_true",
                           dest="math_check_indexes",
                           help="enable checking of math indexes")
    group_build.add_option("--postype-single",
                           default=False,
                           action="store_true",
                           dest="postype_single",
                           help="force single precision copter position controller")
    group_build.add_option("--ekf-single",
                           default=False,
                           action="store_true",
                           dest="ekf_single",
                           help="force single precision EKF")
    group_build.add_option("--force-32bit",
                           default=False,
                           action='store_true',
                           dest="force_32bit",
                           help="compile sitl using 32-bit")
    group_build.add_option("", "--ubsan",
                           default=False,
                           action='store_true',
                           dest="ubsan",
                           help="compile sitl with undefined behaviour sanitiser")
    group_build.add_option("", "--ubsan-abort",
                           default=False,
                           action='store_true',
                           dest="ubsan_abort",
                           help="compile sitl with undefined behaviour sanitiser and abort on error")
    group_build.add_option("--num-aux-imus",
                           dest="num_aux_imus",
                           default=0,
                           type='int',
                           help='number of auxiliary IMUs to simulate')
    group_build.add_option("--enable-dronecan-tests",
                           default=False,
                           action='store_true',
                           dest="dronecan_tests",
                           help="enable dronecan tests")
    parser.add_option_group(group_build)

    group_sim = optparse.OptionGroup(parser, "Simulation options")
    group_sim.add_option("--speedup",
                         default=None,
                         type='int',
                         help='speedup to run the simulations at')
    group_sim.add_option("--valgrind",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under valgrind')
    group_sim.add_option("", "--callgrind",
                         action='store_true',
                         default=False,
                         help="enable valgrind for performance analysis (slow!!)")
    group_sim.add_option("--gdb",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under gdb')
    group_sim.add_option("--gdb-no-tui",
                         default=False,
                         action='store_true',
                         help='when running under GDB do NOT start in TUI mode')
    group_sim.add_option("--gdbserver",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under gdbserver')
    group_sim.add_option("--lldb",
                         default=False,
                         action='store_true',
                         help='run ArduPilot binaries under lldb')
    group_sim.add_option("-B", "--breakpoint",
                         type='string',
                         action="append",
                         default=[],
                         help="add a breakpoint at given location in debugger")
    group_sim.add_option("--disable-breakpoints",
                         default=False,
                         action='store_true',
                         help="disable all breakpoints before starting")
    group_sim.add_option("", "--force-ahrs-type",
                         dest="force_ahrs_type",
                         default=None,
                         help="force a specific AHRS type (e.g. 10 for SITL-ekf")
    group_sim.add_option("", "--replay",
                         action='store_true',
                         help="enable replay logging for tests")
    parser.add_option_group(group_sim)

    group_completion = optparse.OptionGroup(parser, "Completion helpers")
    group_completion.add_option("--list-vehicles",
                                action='store_true',
                                default=False,
                                help='list available vehicles')
    group_completion.add_option("--list-vehicles-test",
                                action='store_true',
                                default=False,
                                help='list available vehicle tester')
    group_completion.add_option("--list-subtests-for-vehicle",
                                type='string',
                                default="",
                                help='list available subtests for a vehicle e.g Copter')
    group_completion.add_option("--reset-after-every-test",
                                action='store_true',
                                default=False,
                                help='reset everything after every test run')
    parser.add_option_group(group_completion)

    opts, args = parser.parse_args()

    # canonicalise on opts.debug:
    if opts.debug is None and opts.no_debug is None:
        # default is to create debug SITL binaries
        opts.debug = True
    elif opts.debug is not None and opts.no_debug is not None:
        if opts.debug == opts.no_debug:
            raise ValueError("no_debug != !debug")
    elif opts.no_debug is not None:
        opts.debug = not opts.no_debug

    if opts.timeout is None:
        opts.timeout = 5400
        # adjust if we're running in a regime which may slow us down e.g. Valgrind
        if opts.valgrind:
            opts.timeout *= 10
        elif opts.callgrind:
            opts.timeout *= 10
        elif opts.gdb:
            opts.timeout = None

    steps = [
        'prerequisites',
        'build.Binaries',
        'build.All',
        'build.Parameters',

        'build.Replay',

        'build.unit_tests',
        'run.unit_tests',
        'build.examples',
        'run.examples',

        'build.Plane',
        'test.Plane',
        'test.QuadPlane',

        'build.Rover',
        'test.Rover',
        'test.BalanceBot',
        'test.Sailboat',

        'build.Copter',
        'test.Copter',

        'build.Helicopter',
        'test.Helicopter',

        'build.Tracker',
        'test.Tracker',

        'build.Sub',
        'test.Sub',

        'build.Blimp',
        'test.Blimp',

        'build.SITLPeriphUniversal',
        'test.CAN',

        'build.SITLPeriphBattMon',
        'test.BattCAN',

        # convertgps disabled as it takes 5 hours
        # 'convertgpx',
    ]

    moresteps = [
        'test.CopterTests1a',
        'test.CopterTests1b',
        'test.CopterTests1c',
        'test.CopterTests1d',
        'test.CopterTests1e',

        'test.CopterTests2a',
        'test.CopterTests2b',

        'clang-scan-build',
    ]

    # canonicalise the step names.  This allows
    # backwards-compatability from the hodge-podge
    # fly.ArduCopter/drive.APMrover2 to the more common test.Copter
    # test.Rover
    step_mapping = {
        "build.ArduPlane": "build.Plane",
        "build.ArduCopter": "build.Copter",
        "build.APMrover2": "build.Rover",
        "build.ArduSub": "build.Sub",
        "build.AntennaTracker": "build.Tracker",
        "fly.ArduCopter": "test.Copter",
        "fly.ArduPlane": "test.Plane",
        "fly.QuadPlane": "test.QuadPlane",
        "dive.ArduSub": "test.Sub",
        "drive.APMrover2": "test.Rover",
        "drive.BalanceBot": "test.BalanceBot",
        "drive.balancebot": "test.BalanceBot",
        "fly.CopterAVC": "test.Helicopter",
        "test.AntennaTracker": "test.Tracker",
        "fly.ArduCopterTests1a": "test.CopterTests1a",
        "fly.ArduCopterTests1b": "test.CopterTests1b",
        "fly.ArduCopterTests1c": "test.CopterTests1c",
        "fly.ArduCopterTests1d": "test.CopterTests1d",
        "fly.ArduCopterTests1e": "test.CopterTests1e",

        "fly.ArduCopterTests2a": "test.CopterTests2a",
        "fly.ArduCopterTests2b": "test.CopterTests2b",

    }

    # form up a list of bits NOT to run, mapping from old step names
    # to new step names as appropriate.
    skipsteps = opts.skip.split(',')
    new_skipsteps = []
    for skipstep in skipsteps:
        if skipstep in step_mapping:
            new_skipsteps.append(step_mapping[skipstep])
        else:
            new_skipsteps.append(skipstep)
    skipsteps = new_skipsteps

    # ensure we catch timeouts
    signal.signal(signal.SIGALRM, alarm_handler)
    if opts.timeout is not None:
        signal.alarm(opts.timeout)

    if opts.list:
        for step in steps:
            print(step)
        sys.exit(0)

    if opts.list_subtests:
        list_subtests()
        sys.exit(0)

    if opts.list_subtests_for_vehicle:
        list_subtests_for_vehicle(opts.list_subtests_for_vehicle)
        sys.exit(0)

    if opts.list_vehicles_test:
        print(' '.join(__bin_names.keys()))
        sys.exit(0)

    if opts.list_vehicles:
        print(' '.join(vehicle_list))
        sys.exit(0)

    util.mkdir_p(buildlogs_dirpath())

    lckfile = buildlogs_path('autotest.lck')
    print("lckfile=%s" % repr(lckfile))
    lck = util.lock_file(lckfile)

    if lck is None:
        print("autotest is locked - exiting.  lckfile=(%s)" % (lckfile,))
        sys.exit(0)

    atexit.register(util.pexpect_close_all)

    # provide backwards-compatability from (e.g.) drive.APMrover2 -> test.Rover
    newargs = []
    for arg in args:
        for _from, to in step_mapping.items():
            arg = re.sub("^%s" % _from, to, arg)
        newargs.append(arg)
    args = newargs

    if len(args) == 0 and not opts.autotest_server:
        print("Steps must be supplied; try --list and/or --list-subtests or --help")
        sys.exit(1)

    if len(args) > 0:
        # allow a wildcard list of steps
        matched = []
        for a in args:
            matches = [step for step in steps
                       if fnmatch.fnmatch(step.lower(), a.lower())]
            x = find_specific_test_to_run(a)
            if x is not None:
                matches.append(x)

            if a in moresteps:
                matches.append(a)

            if not len(matches):
                print("No steps matched {}".format(a))
                sys.exit(1)
            matched.extend(matches)
        steps = matched

    # skip steps according to --skip option:
    steps_to_run = [s for s in steps if should_run_step(s)]

    results = TestResults()

    try:
        if not run_tests(steps_to_run):
            sys.exit(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught; closing pexpect connections")
        util.pexpect_close_all()
        raise
    except Exception:
        # make sure we kill off any children
        util.pexpect_close_all()
        raise
