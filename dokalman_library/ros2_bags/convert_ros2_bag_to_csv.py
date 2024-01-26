from pathlib import Path

from rosbags.dataframe import get_dataframe
from rosbags.highlevel import AnyReader

with AnyReader([Path('my_bag')]) as reader:
    dataframe = get_dataframe(reader, '/cmd_vel', ['linear', 'angular'])