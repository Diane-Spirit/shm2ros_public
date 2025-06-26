#!/usr/bin python3

import os
import mmap
import time
import struct
import numpy as np

'''
Counter: int32 (4 Byte)
Linear velocity X: float64 (8 Byte)
Linear velocity Y: float64 (8 Byte)
Linear velocity Z: float64 (8 Byte)
Angular velocity X: float64 (8 Byte)
Angular velocity Y: float64 (8 Byte)
Angular velocity Z: float64 (8 Byte)
'''

SHARED_CTRL = "/dev/shm/shared_control"
SHARED_CTRL_SIZE = 52 #bits

SEMAPHORE_READ = "/dev/shm/semaphore_ros_read"
SEMAPHORE_SIZE = 5

def create_shm_file(path, size):
    with open(path, "wb") as f:
        # f.write(b"\x00" * size)
        f.truncate(size)
    print(" File ", path, "has been created with size: ", size)

def create_shared_memory(path_size_list):
    for path, size in path_size_list:
        create_shm_file(path, size)

def map_shared_memory(shared_file=SHARED_CTRL, shared_file_size=SHARED_CTRL_SIZE, shared_semaphore=SEMAPHORE_READ, shared_semaphore_size=SEMAPHORE_SIZE):
    path_size_list = [
        (shared_file, shared_file_size),
        (shared_semaphore, shared_semaphore_size),
    ]
    create_shared_memory(path_size_list)  
    
    shared_ctrl = open(SHARED_CTRL, "r+b")
    semaphore_read = open(SEMAPHORE_READ, "r+b")

    shared_mem = mmap.mmap(shared_ctrl.fileno(), SHARED_CTRL_SIZE, 
                           mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    flag_read_mem = mmap.mmap(semaphore_read.fileno(), SEMAPHORE_SIZE, 
                              mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)

    return shared_mem, flag_read_mem

if __name__ == "__main__":

    shared_mem, flag_read_mem = map_shared_memory()

    # velocities
    lin_vel_x = 0.5
    lin_vel_y = 0.
    lin_vel_z = 0.
    ang_vel_x = 0.
    ang_vel_y = 0.
    ang_vel_z = 0.5

    SMALL_STEPS=10
    ST = 1/SMALL_STEPS  # sampling time

    count = 0
    limit = 1000
    while True:
        lin_vel_x = np.clip(lin_vel_x + np.random.uniform(-1, 1), -1, 1)
        ang_vel_z = np.clip(ang_vel_z + np.random.uniform(-1, 1), -1, 1)
        count += 1

        data = [count, lin_vel_x, lin_vel_y, lin_vel_z, ang_vel_x, ang_vel_y, ang_vel_z]
        data_packed = struct.pack("<idddddd", *data)

        print("\n Writing data to SHM: ", data)
        shared_mem.seek(0)
        shared_mem.write(data_packed)

        shared_mem.seek(0)
        data_read = struct.unpack("<idddddd", shared_mem.read(SHARED_CTRL_SIZE))
        print(" Data: ", data_read)
        # input()

        time.sleep(ST)
        if count >= limit:
            break
    
    shared_mem.close()
    flag_read_mem.close()
    print(" Exiting.")