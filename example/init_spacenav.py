import ctypes
from ctypes import *

spnav = ctypes.CDLL("/libspnav/libspnav.so.0.4")


# 필요한 함수들을 선언합니다. 예를 들어, 민감도 설정을 위한 함수들:
spnav.spnav_open.restype = ctypes.c_int
spnav.spnav_close.restype = ctypes.c_int
spnav.spnav_cfg_set_swapyz.restype = ctypes.c_int
spnav.spnav_cfg_set_deadzone.restype = ctypes.c_int

def set_swap_yz(swap):
    """
    spnav_cfg_set_swapyz 함수를 호출하여 Z와 Y 축을 교환할지 설정합니다.
    swap: 1이면 교환, 0이면 비활성화
    """
    if spnav.spnav_open() == -1:
        print("Failed to open connection to spacenavd")
        return False
    
    # spnav_cfg_set_swapyz 함수 호출 (1로 설정하면 Y와 Z 축을 교환)
    result = spnav.spnav_cfg_set_swapyz(ctypes.c_int(swap))
    
    if result == -1:
        print("Failed to set swap YZ")
        spnav.spnav_close()
        return False

    # 설정된 값을 확인하려면 spnav_cfg_get_swapyz 함수도 호출해야 합니다.
    current_swap = spnav.spnav_cfg_get_swapyz()
    if current_swap == -1:
        print("Failed to get swap YZ status")
        spnav.spnav_close()
        return False

    print(f"Swap YZ set to: {current_swap}")

    # 설정을 저장합니다.
    if spnav.spnav_cfg_save() == -1:
        print("Failed to save configuration")
        spnav.spnav_close()
        return False

    # spnav.spnav_close()
    return True

def set_deadzone(val):

    for i in range(6):
        result = spnav.spnav_cfg_set_deadzone(i, val)

    if result == -1:
        print("Failed to dead zone setting")
        spnav.spnav_close()
        return False

    current_deadzone = [0] * 6  # deadzone 값을 저장할 리스트

    for i in range(6):
        current_deadzone[i] = spnav.spnav_cfg_get_deadzone(i)

        if current_deadzone[i] == -1:
            print("Failed to get set deadzone status")
            spnav.spnav_close()
            return False

        print(f" set deadzone to: {current_deadzone[i]}")

    # 설정을 저장합니다.
    if spnav.spnav_cfg_save() == -1:
        print("Failed to save configuration")
        spnav.spnav_close()
        return False

    # spnav.spnav_close()
    return True
# 사용 예시: Y와 Z 축을 교환 활성화 (1)
set_swap_yz(1)
set_deadzone(200)