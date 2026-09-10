# Pulled from ECR Public, not Docker Hub. `public.ecr.aws/docker/library/ros` is
# the ECR Public mirror of the same official ROS image — the digest below is
# byte-identical to Docker Hub's `ros:humble-ros-base`, so this is a registry
# change and not an image change. ECR Public also removes Docker Hub's anonymous
# pull rate limit, which is the failure people actually hit when rebuilding.
#
# Pinned by DIGEST, not just by tag. `humble-ros-base` is mutable — it is rebuilt
# whenever the upstream ROS image is, so two builds of the same commit could
# differ in every apt package below and there would be nothing in the repo
# recording which one shipped. The digest makes the base a fixed input.
#
# The tag is kept alongside it for readability; Docker resolves the digest and
# ignores the tag. Both are recorded in THIRD-PARTY-LICENSES.
#
# To move to a newer base deliberately:
#   docker pull public.ecr.aws/docker/library/ros:humble-ros-base
#   docker inspect --format='{{index .RepoDigests 0}}' \
#     public.ecr.aws/docker/library/ros:humble-ros-base
FROM public.ecr.aws/docker/library/ros:humble-ros-base@sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987

# Install basic requirements
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    portaudio19-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    ros-humble-rviz2 \
    ros-humble-vision-msgs \
    software-properties-common \
    ros-humble-rmw-cyclonedds-cpp \
    ffmpeg

# NumPy first, and exactly. `numpy<2.0.0` had no lower bound, so this layer could
# resolve to any 1.x release — including ones predating the wheels the ROS 2
# Humble Python packages are built against. Same version as requirements.in, on
# purpose: two pins that can disagree eventually do.
RUN pip install --no-cache-dir 'numpy==1.26.4'

# Create workspace
WORKDIR /ros2_ws

# Install Python requirements FIRST, before copying the full source tree.
# The closure changes rarely, so this heavy layer (incl. torch, ~1-4 GB depending
# on arch) stays cached across code-only edits — only colcon re-runs.
#
# These are the COMPILED closures (uv pip compile, via `make lock`), not lists of
# direct deps: every transitive version is fixed here rather than re-resolved at
# build time. They are also the files `make audit` scans, which is the point —
# what gets scanned is what gets installed.
#
# TWO closures, one per architecture, because `torch==2.14.0` on PyPI is a
# different BUILD per arch and only one of them runs where it lands:
#
#   requirements.txt        x86_64  — CUDA torch. What the EC2 target wants: the
#                                     instance is a g4dn (T4) and it launches the
#                                     detector with coco_device:=cuda. This is the
#                                     file `make push-image` (--platform
#                                     linux/amd64) builds with.
#   requirements-arm64.txt  aarch64 — torch 2.14.0+cpu. PyPI's aarch64 wheel is the
#                                     Grace/sbsa CUDA build, whose NVPL-linked
#                                     libtorch_cpu.so SEGFAULTS at dlopen on Apple
#                                     Silicon under Docker Desktop, killing
#                                     coco_detector_node at startup (exit -11,
#                                     before it logs anything) — no
#                                     /annotated_image, no boxes in Lichtblick.
#                                     Also drops ~3.3 GB of unusable nvidia-*
#                                     wheels from the image.
#
# See requirements-arm64.in for the full story. Both files are hash-pinned and both
# are installed with --require-hashes, which additionally refuses anything in them
# that is not pinned.
COPY requirements.txt requirements-arm64.txt /ros2_ws/
RUN --mount=type=cache,target=/root/.cache/pip \
    if [ "$(uname -m)" = "aarch64" ]; then REQ=requirements-arm64.txt; \
    else REQ=requirements.txt; fi && \
    echo "[*] installing $REQ ($(uname -m))" && \
    pip install --require-hashes --default-timeout=300 --retries 20 \
        -r "/ros2_ws/$REQ"

# Smoke-test the import that has broken twice now (see requirements-arm64.in).
# `import torch` failing is invisible at build time otherwise: it surfaces much
# later as coco_detector_node dying with SIGSEGV and no log line, which reads like
# a camera or QoS problem rather than a wheel problem.
RUN python3 -c "import torch, torchvision; print('torch', torch.__version__, '/ torchvision', torchvision.__version__)"

# Now copy the source. Edits below this line don't bust the pip layer above.
COPY . /ros2_ws/src

# Install ROS dependencies
# Refresh apt lists + rosdep cache first: the base image ships stale apt
# indexes, and the ROS apt repo drops superseded .deb versions, so rosdep's
# `apt-get install` 404s (and exits 1) unless we update right before it.
RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install --from-paths src --ignore-src -r -y

# Downgrade setuptools/packaging right before the build. requirements.txt now pins
# them explicitly in its closure (setuptools==84.0.0, packaging==26.3), and
# packaging's canonicalize_version() added the strip_trailing_zero kwarg — colcon's
# setup.py parsing passes it and fails with "TypeError: canonicalize_version() got
# an unexpected keyword argument 'strip_trailing_zero'". Pinning here (after all
# other pip installs) is the only spot that sticks; an earlier pin gets clobbered
# by requirements.txt.
#
# Exact versions, not `<66` / `<22`. Those were open at the bottom, so the layer
# could resolve to anything old enough — including setuptools before 65.5.1, which
# carries CVE-2022-40897 (ReDoS in package_index).
#
# KNOWN, ACCEPTED FINDING — a scanner will flag this line, so here is the analysis.
# 65.5.1 is the newest release compatible with colcon's setup.py parsing, and three
# later setuptools CVEs therefore cannot be fixed by moving the pin:
#
#   CVE-2024-6345  (HIGH, fixed 70.0.0)  command injection via package URL
#   CVE-2025-47273 (HIGH, fixed 78.1.1)  path traversal in PackageIndex.download
#   CVE-2026-59890 (MOD,  fixed 83.0.0)  MANIFEST.in bypass when building an sdist
#
# All three are in `setuptools.package_index` / sdist building — code reached only
# when setuptools DOWNLOADS from an index or BUILDS a source distribution. This
# image does neither: every pip install completed in the layers above, this pin
# exists solely so `colcon build` can parse setup.py, and nothing invokes setuptools
# at runtime. The vulnerable paths are unreachable here.
#
# Path to production: upgrade setuptools in a final layer, AFTER colcon build, so
# the shipped image carries a current version while the build still gets 65.5.1.
# Deliberately not done here because it is unverified against `ros2 run`'s
# entry-point resolution and this image takes a 14 GB build to test.
RUN pip install --no-cache-dir 'setuptools==65.5.1' 'packaging==21.3'

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

# Source ROS2 and workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set environment variables
ENV ROBOT_IP=""
ENV CONN_TYPE="webrtc"
ENV WEBRTC_SERVER_HOST="0.0.0.0"
ENV WEBRTC_SERVER_PORT="9991"

# Remove apt cache
RUN rm -rf /var/lib/apt/lists/*

# add /ros2_ws to the entrypoint
RUN sed --in-place --expression \
      '$isource "/ros2_ws/install/setup.bash"' \
      /ros_entrypoint.sh

CMD ["ros2", "launch", "go2_robot_sdk", "robot.launch.py"]
