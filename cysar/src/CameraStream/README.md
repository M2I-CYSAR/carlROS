# CySAR - CameraStream
## Description
This is a utility for hosting USB camera streams via [mjpeg-streamer](https://github.com/jacksonliam/mjpg-streamer) on [alpine linux docker containers](https://hub.docker.com/_/alpine). If cameras are unplugged, the stream should automatically stop and, when re-plugged, be restarted.

## Usage
The camera_stream.py can be executed directly or used as a library. Most importantly, at the top of the file, you can specify which cameras to host and on what ports. Once running, access the stream at 
`http://[2610:130:110:1525:47e7:9414:7e67:15e4]:5000/?action=stream`,
where 5000 is the port number.

## Docker
Hosting camera streams is inherently an unstable process, so I containerized each camera stream inside docker containers such that if they crashed, it could in no way harm the overall system. If you are not familiar with docker and need to change this area of code, I recommend looking up videos or tutorials online. 