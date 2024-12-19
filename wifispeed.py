#!/usr/bin/python3
# SPDX-FileCopyrightText: 2024 Abdelrahman Alhanbali <abdelrahman.alhanbali@gmail.com>

import speedtest
import math

def bytes_to_mb(size_bytes):
    i = int(math.floor(math.log(size_bytes, 1024))) #size_bytesを1024で対数をとる。KB → MB → GB
    power = math.pow(1024, i) 	#pow(4,3)で4の3乗
    size = round(size_bytes / power, 2) #小数点以下を2桁に丸める。
    return f"{size} Mbps"

wifi = speedtest.Speedtest()

print("Getting download speed...")
download_speed = wifi.download()

print("Getting upload speed...")
upload_speed = wifi.upload()

print("Download wifi speed : ",  bytes_to_mb(download_speed))
print("Upload wifi speed : ", bytes_to_mb(upload_speed))
