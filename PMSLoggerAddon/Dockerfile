ARG BUILD_FROM
FROM $BUILD_FROM

# Install requirements for add-on
RUN \
  apk add --no-cache \
    python3 \
    py3-pip

RUN \
  pip install --no-cache-dir paho-mqtt \
                             peewee

# Copy data for add-on
COPY rootfs /
