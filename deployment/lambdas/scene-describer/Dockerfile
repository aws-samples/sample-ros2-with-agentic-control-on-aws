# Pinned by digest so two builds of one commit produce the same image — see the
# note in docker/Dockerfile. AWS rebuilds the :3.11 tag for patch updates, so the
# tag alone is a moving input. Recorded in THIRD-PARTY-LICENSES.
FROM public.ecr.aws/lambda/python:3.11@sha256:755093e1da2ba33f6bd2295c5a6a7ed8b94f1ef347b6ca649874d9cbef5a3e3c

# Install imageio-ffmpeg to get the static ffmpeg binary. Pinned for the same
# reason every other dependency in this repo is: an unpinned build here changes
# the ffmpeg that cuts frames out of the KVS clip, with no record of when.
RUN pip install --no-cache-dir 'imageio-ffmpeg==0.6.0' && \
    cp $(python3 -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())") /usr/local/bin/ffmpeg && \
    chmod +x /usr/local/bin/ffmpeg

# Copy shared utilities
COPY shared/python/ ${LAMBDA_TASK_ROOT}/

# Copy function code
COPY scene-describer/index.py ${LAMBDA_TASK_ROOT}/

CMD ["index.handler"]
