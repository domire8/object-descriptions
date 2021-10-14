ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG USERNAME
ARG BUILD_COMMAND
USER ${USERNAME}
WORKDIR ${HOME}/${USERNAME}_ws
COPY --chown=${USER} ./object_descriptions ./src/object_descriptions

RUN /bin/bash -c "${BUILD_COMMAND}"