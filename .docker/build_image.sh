#!/usr/bin/env bash

echo -e "Building moveit1_ur:lastest image"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--target bash \
--tag moveit1_ur:latest .