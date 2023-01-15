# syntax=docker/dockerfile:1

FROM python:3.8-slim-buster

ENV DEBIAN_FRONTEND=noninteractive TZ="Europe/Berlin" 

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update -y && apt-get install --no-install-recommends -y apt-utils

WORKDIR /

RUN pip install --upgrade pip

RUN apt-get install libglib2.0-0 -y

RUN apt-get install tk -y

COPY requirements.txt requirements.txt

RUN pip3 install -r requirements.txt


RUN pip install pyrr \
 opencv-python \
 # Pillow \
 scipy \
 sympy \
 matplotlib 

RUN python -m pip install numpy-quaternion


COPY /code  /code
COPY /maps  /maps
COPY /result  /result
COPY requirements.txt requirements.txt

RUN chmod 777  result/result.csv

RUN chmod 777  result/result.svg

CMD [ "sh","-c","python3  code/main.py" ] 
