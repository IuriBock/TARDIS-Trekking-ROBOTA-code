import bluerobotics_navigator as navigator

navigator.init()
sensors_ok = navigator.self_test()
print(sensors_ok)