import os
import glob

def list_tty_acm():
    """列出所有 /dev/ttyACM* 设备"""
    devices = glob.glob("/dev/ttyACM*")
    if not devices:
        print("未找到 /dev/ttyACM 设备！")
        return None
    print("检测到以下 /dev/ttyACM 设备：")
    for i, dev in enumerate(devices):
        print(f"{i}: {dev}")
    return devices

def select_device(devices):
    """让用户选择一个设备"""
    while True:
        try:
            index = int(input("请输入设备编号："))
            if 0 <= index < len(devices):
                # 将index作为devices的第一位，剩余进行排序
                if index != 0:
                    devices = [devices[index]] + devices[:index] + devices[index+1:]
                else:
                    devices = devices
                # 输出devices
                print(f"选择的设备为：{devices[0]}")
                return devices
            else:
                print("编号超出范围，请重新输入！")
        except ValueError:
            print("输入无效，请输入数字！")

def create_symlink(target):
    """为每个设备创建符号链接 /dev/sagittarius_0, /dev/sagittarius_1, ..."""
    idx = 0
    for idx, dev in enumerate(target):
        symlink_path = f"/dev/sagittarius_{idx}"
        # print(f"symlink_path:{symlink_path}")
        if os.path.islink(symlink_path) or os.path.exists(symlink_path):
            os.remove(symlink_path)  # 删除旧的符号链接
        os.symlink(dev, symlink_path)
        print(f"已将 {target} 映射为 {symlink_path}")

def main():
    devices = list_tty_acm()
    if not devices:
        return
    selected_device = select_device(devices)
    create_symlink(selected_device)

if __name__ == "__main__":
    main()
