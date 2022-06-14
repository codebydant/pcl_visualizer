FROM danieltobon43/pcl-docker:1.12.1-alpine3.15-dev AS build
COPY . /tmp
WORKDIR /tmp
RUN cmake -B `pwd`/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install && \
    make -j$(nproc) -C build/ --no-print-directory && make install -C build/

FROM danieltobon43/pcl-docker:1.12.1-alpine3.15 AS runtime
COPY --from=build /tmp/install /usr
ENTRYPOINT ["pcl-visualizer"]