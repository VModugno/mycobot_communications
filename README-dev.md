# dev README

Notes for developpers...

# pymycobot version
by default on the pi's with Ubuntu 18 we have pymycobot==2.7.5 installed and python 2.
this is quite old and slow.

## useful_scripts/speed_tests.py

```
python_version '2.7.17 (default, Feb 27 2021, 15:10:58) \n[GCC 7.5.0]'
pymycobot_version '2.7.5'
average timings
{'get_angles': 0.1082292530271742, 'get_encoders': 0.1065657999780443}
number of calls
{'get_angles': 36, 'get_encoders': 36}
```

If we use python3 and a newer version of pymycobot (this is the criticial bit) we get way better performance.
```
python_version '3.8.10 (default, Jul 29 2024, 17:02:10) \n[GCC 9.4.0]'
pymycobot_version '3.4.9'
robot_version: 1
sys_version: 3.1
robo_id: 0
basic_firmware_version: None
atom_version: None
average timings
{'get_angles': 0.009076217752048346, 'get_encoders': 0.008493457721475282}
number of calls
{'get_angles': 341, 'get_encoders': 341}
```
## useful_scripts/speed_tests.py
Because we synchronize reads/writes, if we are reading at 100HZ but can only write at 50HZ, we limit both reads and writes to 50HZ. We are very interested in whether we can read/write without synchronizing, and the concurrency_test.py tries to test this.

If we use multiprocessing and try and use the serial port from multiple processes it throws a `serial.serialutil.SerialException` warning about access from multiple processes. This is not unexpected given some of the notes [here](https://stackoverflow.com/questions/30316722/what-is-the-best-practice-for-locking-serial-ports-and-other-devices-in-linux). When we use python threading (which doesn't do 2 things at the same time given the global interpreter lock) we can run the program but performance is worse than it was previously.

```
root@er:/mnt_folder/mycobot_communications/useful_scripts# python3 concurrency_test.py
sending command
getting angles
Process Process-1:
Traceback (most recent call last):
  File "/usr/lib/python3.10/multiprocessing/process.py", line 314, in _bootstrap
    self.run()
  File "/usr/lib/python3.10/multiprocessing/process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  File "/mnt_folder/mycobot_communications/useful_scripts/concurrency_test.py", line 86, in get_angles
    angles = self.mc.get_angles()
  File "/usr/local/lib/python3.10/dist-packages/pymycobot/generate.py", line 199, in get_angles
    return self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)
  File "/usr/local/lib/python3.10/dist-packages/pymycobot/mycobot.py", line 93, in _mesg
    return self._res(real_command, has_reply, genre)
  File "/usr/local/lib/python3.10/dist-packages/pymycobot/mycobot.py", line 100, in _res
    data = self._read(genre, _class=self.__class__.__name__)
  File "/usr/local/lib/python3.10/dist-packages/pymycobot/common.py", line 727, in read
    data = self._serial_port.read()
  File "/usr/local/lib/python3.10/dist-packages/serial/serialposix.py", line 595, in read
    raise SerialException(
serial.serialutil.SerialException: device reports readiness to read but returned no data (device disconnected or multiple access on port?)
```

# notes on docker

Some of us use docker. You can use the Dockerfile here for ros-noetic.

```
sudo docker build -t mycobot-noetic -f Dockerfile .
sudo docker run -it --network host --device /dev/ttyAMA0 --volume /home/ubuntu/catkin_ws/src/:/mnt_folder mycobot-noetic
source devel/setup.bash
roslaunch mycobot_interface communication_topic.launch
```

# notes on firmware upgrades
Atom version we will update with my studio. Use the my studio on the pi. The mystudio that we can download from the current website doesn't seem to have atom version available, just M5. Plug the USB C cable into the top atom port, and then into the pi. This is launchable from the desktop of the pi.

```
https://docs.elephantrobotics.com/docs/gitbook-en/4-BasicApplication/4.1-myStudio/
PI	RaspberryPI 4B	ubuntu	v18.04.is recommened
Atom	atomMain	v4.1 is recommended for robots labelled ER28001202200415 and before，or not lablled; v5.1 is recommended for robots lablled ER28001202200416 and after
```