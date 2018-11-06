# specify the linux base image with your desired version ubuntu:<version>
FROM ubuntu:14.04

# Initial internal configuration.
RUN apt-get -y update
RUN apt-get -y install sudo iputils-ping lsb-core software-properties-common python-dev python-pip
RUN apt-get -y update
RUN apt-get -y install build-essential software-properties-common
RUN pip install -i https://pypi.python.org/simple/ --upgrade pip==9.0
RUN pip install --upgrade --force-reinstall pyserial
RUN pip install future lxml && pip install pymavlink MAVProxy

# Docker folder to work.
WORKDIR /app
RUN mkdir -p Tools/scripts/
COPY Tools/scripts/install-prereqs-ubuntu.sh ./Tools/scripts/

# Prereqs for build Ardupilot.
ENV USER=root
RUN ./Tools/scripts/install-prereqs-ubuntu.sh -y

RUN echo "export PATH=/opt/gcc-arm-none-eabi-4_9-2015q3/bin:\$PATH" >>  ~/.bashrc
RUN echo "export PATH=/usr/lib/ccache:\$PATH:/app/Developer/Drone/Tools/autotest" >>  ~/.bashrc

#CMD bash
#CMD ./waf configure --board bebop --static && ./waf copter
CMD cd Developer/Drone && . ~/.bashrc && \
    ./waf configure --board mini-pix && ./waf copter
