import os
import glob

def list_tty_acm():
    """列出所有 /dev/video* 设备"""
    devices = glob.glob("/dev/video*")
    if not devices:
        print("未找到 /dev/video 设备！")
        return None
    print("检测到以下 /dev/video 设备：")
    for i, dev in enumerate(devices):
        print(f"{i}: {dev}")
    return devices

def select_device(devices):
    """让用户选择一个设备"""
    while True:
        try:
            index = int(input("请输入设备编号："))
            if 0 <= index < len(devices):
                return devices[index]
            else:
                print("编号超出范围，请重新输入！")
        except ValueError:
            print("输入无效，请输入数字！")

def create_symlink(target):
    """创建符号链接 /dev/usbcam"""
    symlink_path = "/dev/usbcam"
    target_name = os.path.basename(target)  # 只保留 "videoX"
    if os.path.exists(symlink_path):
        os.remove(symlink_path)  # 删除旧的符号链接
    os.symlink(target_name, symlink_path)
    print(f"已将 {target} 映射为 {symlink_path}")

def main():
    devices = list_tty_acm()
    if not devices:
        return
    selected_device = select_device(devices)
    create_symlink(selected_device)

if __name__ == "__main__":
    main()
