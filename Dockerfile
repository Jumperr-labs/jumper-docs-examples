FROM python:2.7-stretch
ENTRYPOINT /bin/bash
ENV DOCKER_JUMPER=1
RUN apt-get update && apt-get install -y gcc-arm-none-eabi
RUN pip install jumper
